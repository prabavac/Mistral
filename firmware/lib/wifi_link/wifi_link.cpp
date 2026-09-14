#include "wifi_link.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <mistral_config.h>

// The ground-station page. web/index.html is its only copy: board_build.embed_txtfiles in
// platformio.ini links it into the image, and it is served straight from flash — never
// rebuilt per request. The embed appends a NUL terminator, which is not sent. Declared
// outside the anonymous namespace so the names keep external linkage.
extern const uint8_t PAGE_START[] asm("_binary_web_index_html_start");
extern const uint8_t PAGE_END[] asm("_binary_web_index_html_end");

namespace {

AsyncWebServer server(cfg::WIFI_HTTP_PORT);
AsyncWebSocket ws(cfg::WIFI_WS_PATH);

// Guards everything below, shared between core 0 (network) and core 1 (control loop).
// Held only for copies.
portMUX_TYPE         lock = portMUX_INITIALIZER_UNLOCKED;
wifi_link::Commands  commands{};
wifi_link::Telemetry telemetry{};
uint32_t             lastRxMs = 0;      // millis() of the last valid frame from any client —
bool                 haveRx   = false;  // the link-loss heartbeat

// Acks waiting for the publisher task. Queueing never blocks: a full queue drops the ack.
struct Ack {
    uint32_t id;
    bool     ok;
    char     error[64];
};
constexpr UBaseType_t ACK_QUEUE_LEN = 16;
QueueHandle_t         acks          = nullptr;

void queueAck(uint32_t id, bool ok, const char* error) {
    Ack a{id, ok, {}};
    if (!ok && error) strlcpy(a.error, error, sizeof a.error);
    xQueueSend(acks, &a, 0);
}

// Every event command (kill, arm, disarm) also drops the held throttle to 0, in the same
// critical section, so each flight starts from zero and a stale slider value can never
// block re-arming or survive a kill.
void setEvent(bool& flag, uint32_t& idField, uint32_t id) {
    portENTER_CRITICAL(&lock);
    flag              = true;
    idField           = id;
    commands.throttle = 0.0f;
    portEXIT_CRITICAL(&lock);
}

// Runs in the AsyncTCP task. Records one command; never touches the actuators.
void handleCommand(const uint8_t* data, size_t len) {
    JsonDocument doc;
    if (deserializeJson(doc, data, len)) {
        queueAck(0, false, "malformed JSON");
        return;
    }
    portENTER_CRITICAL(&lock);
    lastRxMs = millis();
    haveRx   = true;
    portEXIT_CRITICAL(&lock);

    const char*    type = doc["type"] | "";
    const uint32_t id   = doc["id"] | 0u;

    if (strcmp(type, "ping") == 0) {
        return;  // heartbeat only — no ack
    } else if (strcmp(type, "kill") == 0) {
        setEvent(commands.kill, commands.killId, id);
    } else if (strcmp(type, "arm") == 0) {
        setEvent(commands.arm, commands.armId, id);
    } else if (strcmp(type, "fly") == 0) {
        setEvent(commands.fly, commands.flyId, id);
    } else if (strcmp(type, "disarm") == 0) {
        setEvent(commands.disarm, commands.disarmId, id);
    } else if (strcmp(type, "throttle") == 0) {
        JsonVariant v     = doc["value"];
        const float value = v.as<float>();
        if (!(v.is<float>() || v.is<int>()) || !(value >= 0.0f && value <= 1.0f)) {
            queueAck(id, false, "value must be a number 0..1");
            return;
        }
        portENTER_CRITICAL(&lock);
        commands.throttle = value;
        portEXIT_CRITICAL(&lock);
        queueAck(id, true, nullptr);
    } else if (strcmp(type, "trim") == 0) {
        const char* axis = doc["axis"] | "";
        JsonVariant us   = doc["us"];
        const bool  x    = strcmp(axis, "x") == 0;
        const bool  y    = strcmp(axis, "y") == 0;
        if (!(x || y) || !us.is<int16_t>()) {
            queueAck(id, false, "trim needs axis x|y and integer us");
            return;
        }
        portENTER_CRITICAL(&lock);
        (x ? commands.trimXUs : commands.trimYUs) = us.as<int16_t>();
        portEXIT_CRITICAL(&lock);
        queueAck(id, true, nullptr);
    } else {
        queueAck(id, false, "unknown type");
    }
}

void onWsEvent(AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType type, void* arg,
               uint8_t* data, size_t len) {
    if (type != WS_EVT_DATA) return;
    const auto* info = static_cast<AwsFrameInfo*>(arg);
    // Commands are small: accept only complete, single-frame text messages.
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        handleCommand(data, len);
    }
}

const char* stateName(throttle::State s) {
    switch (s) {
        case throttle::State::ARMING: return "ARMING";
        case throttle::State::ARMED:  return "ARMED";
        default:                      return "DISARMED";
    }
}

void sendJson(const JsonDocument& doc) {
    char         buf[768];
    const size_t n = serializeJson(doc, buf, sizeof buf);
    ws.textAll(buf, n);
}

void sendTelemetry() {
    portENTER_CRITICAL(&lock);
    const wifi_link::Telemetry t = telemetry;
    portEXIT_CRITICAL(&lock);

    JsonDocument doc;
    constexpr float R2D = 57.29578f;
    doc["type"]         = "telemetry";
    doc["uptimeMs"]     = t.uptimeMs;
    doc["state"]        = state_machine::name(t.flight);

    JsonObject esc        = doc["throttle"].to<JsonObject>();
    esc["state"]          = stateName(t.esc.state);
    esc["armRemainingMs"] = t.esc.armRemainingMs;
    esc["normalised"]     = t.esc.normalised;
    esc["differential"]   = t.esc.differential;
    esc["esc1Us"]         = t.esc.esc1_us;
    esc["esc2Us"]         = t.esc.esc2_us;
    esc["saturated"]      = t.esc.saturated;

    JsonObject nozzle    = doc["tvc"].to<JsonObject>();
    nozzle["xDeg"]       = t.nozzle.x_rad * R2D;
    nozzle["yDeg"]       = t.nozzle.y_rad * R2D;
    nozzle["xUs"]        = t.nozzle.x_us;
    nozzle["yUs"]        = t.nozzle.y_us;
    nozzle["xTrimUs"]    = t.nozzle.x_trim_us;
    nozzle["yTrimUs"]    = t.nozzle.y_trim_us;
    nozzle["xSaturated"] = t.nozzle.x_saturated;
    nozzle["ySaturated"] = t.nozzle.y_saturated;

    JsonObject att      = doc["att"].to<JsonObject>();
    att["cf1Deg"]       = t.att.comp.r1 * R2D;
    att["cf2Deg"]       = t.att.comp.r2 * R2D;
    att["fu1Deg"]       = t.att.fusion.r1 * R2D;
    att["fu2Deg"]       = t.att.fusion.r2 * R2D;
    att["dr1Dps"]       = t.att.dr1 * R2D;
    att["dr2Dps"]       = t.att.dr2 * R2D;
    att["yawRateDps"]   = t.att.yawRate * R2D;
    att["fusion"]       = cfg::ATTITUDE_USE_FUSION;
    att["valid"]        = t.att.valid;

    JsonObject ctl = doc["ctl"].to<JsonObject>();
    ctl["u1Deg"]   = t.ctl.u1 * R2D;
    ctl["u2Deg"]   = t.ctl.u2 * R2D;
    ctl["ir1"]     = t.ctl.ir1;
    ctl["ir2"]         = t.ctl.ir2;
    ctl["integrating"] = t.integrating;

    sendJson(doc);
}

void sendAck(const Ack& a) {
    JsonDocument doc;
    doc["type"] = "ack";
    doc["id"]   = a.id;
    doc["ok"]   = a.ok;
    if (!a.ok) doc["error"] = a.error;
    sendJson(doc);
}

// Core 0: telemetry at cfg::TELEMETRY_HZ, acks as soon as they are queued.
void publisherTask(void*) {
    const TickType_t period = pdMS_TO_TICKS(1000 / cfg::TELEMETRY_HZ);
    TickType_t       last   = xTaskGetTickCount();
    for (;;) {
        const TickType_t elapsed = xTaskGetTickCount() - last;
        if (elapsed >= period) {
            last = xTaskGetTickCount();
            if (ws.count() > 0) sendTelemetry();
            ws.cleanupClients();
            continue;
        }
        Ack a;
        if (xQueueReceive(acks, &a, period - elapsed) == pdTRUE && ws.count() > 0) {
            sendAck(a);
        }
    }
}

}  // namespace

bool wifi_link::init() {
    acks = xQueueCreate(ACK_QUEUE_LEN, sizeof(Ack));

    WiFi.mode(WIFI_AP);
    uint8_t mac[6];
    WiFi.softAPmacAddress(mac);
    char ssid[32];
    snprintf(ssid, sizeof ssid, "%s%02x%02x", cfg::WIFI_SSID_PREFIX, mac[4], mac[5]);
    if (!WiFi.softAP(ssid, cfg::WIFI_PASSWORD)) return false;

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "text/html", PAGE_START, static_cast<size_t>(PAGE_END - PAGE_START - 1));
    });
    server.begin();

    xTaskCreatePinnedToCore(publisherTask, "wifi_pub", 4096, nullptr, 1, nullptr, cfg::WIFI_CORE);

    Serial.printf("WiFi AP %s  password %s  http://%s\n", ssid, cfg::WIFI_PASSWORD,
                  WiFi.softAPIP().toString().c_str());
    return true;
}

wifi_link::Commands wifi_link::takeCommands() {
    portENTER_CRITICAL(&lock);
    Commands c = commands;
    // Read the clock inside the lock, so a frame landing concurrently can never make
    // lastRxMs newer than `now` and wrap the age.
    const uint32_t now = millis();
    c.linkAlive        = haveRx && now - lastRxMs < cfg::LINK_TIMEOUT_MS;
    commands.kill = commands.arm = commands.fly = commands.disarm = false;
    portEXIT_CRITICAL(&lock);
    return c;
}

void wifi_link::publish(const Telemetry& t) {
    portENTER_CRITICAL(&lock);
    telemetry = t;
    portEXIT_CRITICAL(&lock);
}

void wifi_link::ack(uint32_t id, bool ok, const char* error) { queueAck(id, ok, error); }
