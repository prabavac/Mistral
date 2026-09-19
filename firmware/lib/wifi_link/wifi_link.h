// wifi_link.h — SoftAP + WebSocket ground link — Mistral
//
// The vehicle is the access point: SSID cfg::WIFI_SSID_PREFIX + last 4 hex digits of
// the AP MAC, WPA2 password cfg::WIFI_PASSWORD, page at http://192.168.4.1, WebSocket
// at cfg::WIFI_WS_PATH. No router dependency. The page source is web/index.html at the
// repo root, embedded into the firmware at build time.
//
// THE CONTROL LOOP NEVER BLOCKS ON THE NETWORK. The WebSocket handler (AsyncTCP task)
// and the telemetry publisher task run on core 0 (cfg::WIFI_CORE); the control loop
// runs on core 1. The handler only records commands; the loop polls them with
// takeCommands() at its own rate. publish() and ack() only copy data or queue it
// and return immediately.
//
// LINK LOSS DISARMS. Every valid frame is a heartbeat (the page pings every 250 ms).
// Commands::linkAlive goes false after cfg::LINK_TIMEOUT_MS without one, and loop()
// disarms.
//
// Protocol — JSON text frames, each with a "type".
//
//   client → vehicle (every command except ping carries a numeric "id"):
//     {"type":"ping"}                              heartbeat — no ack
//     {"type":"kill","id":1}
//     {"type":"arm","id":2}                        DISARMED→ARMED (servos), FLYING→ARMED (motors off)
//     {"type":"fly","id":6}                        ARMED→FLYING (ESC arm hold, then throttle)
//     {"type":"disarm","id":3}
//     {"type":"throttle","id":4,"value":0.35}      0..1
//     {"type":"trim","id":5,"axis":"x","us":12}    absolute trim, µs, clamped to ±cfg::TVC_TRIM_MAX_US
//     {"type":"zero","id":8}                       re-zero the IMU: DISARMED and still only, ~1.5 s
//     {"type":"balance","id":9,"ratio":1.02}       static rotor balance, clamped to cfg::ESC_BALANCE_RANGE
//     {"type":"gains","id":7,"scale":0.3,"x":{"kth":1.136,"kq":0.284,"ki":0},"y":{...}}
//                                                  full set, clamped in lqr::setGains
//
//   vehicle → client:
//     {"type":"ack","id":2,"ok":true}
//     {"type":"ack","id":2,"ok":false,"error":"throttle not zero"}
//         kill/arm/fly/disarm/zero are acked by the control loop once applied, with the result.
//         kill/arm/fly/disarm also reset the held throttle to 0 on receipt; zero never does.
//         throttle/trim/gains are acked on receipt (applied next tick). Malformed or unknown
//         commands are rejected on receipt.
//     {"type":"config","gains":{"scale":{"def":..,"min":..,"max":..},
//      "x":{"kth":{"design":..,"def":..,"min":..,"max":..},"kq":{..},"ki":{..}},"y":{..}},
//      "trim":{"maxUs":..,"x":{"centreUs":..,"usPerNozzleDeg":..},"y":{..}},
//      "balance":{"def":..,"min":..,"max":..}}
//         sent to each client as it connects: defaults and allowed ranges for the Tuning panel.
//         usPerNozzleDeg is signed (includes the servo's dir).
//     {"type":"telemetry","uptimeMs":..,"state":"DISARMED|ARMED|FLYING",
//      "throttle":{"state":"DISARMED|ARMING|ARMED","armRemainingMs":..,"normalised":..,
//                  "differential":..,"balance":..,"esc1Us":..,"esc2Us":..,"saturated":..},
//      "tvc":{"xDeg":..,"yDeg":..,"xUs":..,"yUs":..,"xTrimUs":..,"yTrimUs":..,
//             "xSaturated":..,"ySaturated":..},
//      "att":{"cf1Deg":..,"cf2Deg":..,"fu1Deg":..,"fu2Deg":..,"dr1Dps":..,"dr2Dps":..,
//             "yawRateDps":..,"offset1Dps":..,"offset2Dps":..,"fusion":true|false,"valid":..},
//      "ctl":{"u1Deg":..,"u2Deg":..,"ir1":..,"ir2":..,"integrating":..,       ir in rad·s
//             "clampDeg":..,"iLimit":..,"iThrPct":..},   static limits, for the graphs
//      "gains":{"scale":..,"x":[kth,kq,ki],"y":[kth,kq,ki]}}   the gains in use
//         att: r1/r2 from both estimators (display units — the controller is radians);
//         "fusion" says which one drives the controller.
//         pushed to every client at cfg::TELEMETRY_HZ.
#pragma once

#include <cstdint>

#include <mistral_config.h>
#include <attitude.h>
#include <lqr.h>
#include <state_machine.h>
#include <throttle.h>
#include <tvc.h>

namespace wifi_link {

struct Commands {
    // Events: set by a received command, cleared by takeCommands().
    bool     kill, arm, fly, disarm, zero;
    uint32_t killId, armId, flyId, disarmId, zeroId;  // echoed back in the ack
    // Levels: the latest received value, held between takes.
    float   throttle;  // 0..1 — reset to 0 by kill, arm and disarm
    int16_t trimXUs, trimYUs;
    float   balance = cfg::ESC_BALANCE_DEF;  // static rotor balance ratio
    lqr::Gains gains;     // the latest received "gains" command
    uint32_t   gainsSeq;  // incremented per received "gains" command; the loop applies on change
    // Computed by takeCommands(): a valid frame arrived within cfg::LINK_TIMEOUT_MS.
    bool linkAlive;
};

struct Telemetry {
    uint32_t           uptimeMs;
    throttle::Status   esc;
    tvc::Status        nozzle;
    attitude::Estimate   att;
    state_machine::State flight;
    lqr::Output          ctl;
    bool                 integrating;  // integrators live this tick
    lqr::Gains           gains;        // gains in use
};

// Start the SoftAP, HTTP server, WebSocket and the publisher task. Returns false if the
// access point fails to start. Call once in setup().
bool init();

// Copy out the pending commands and clear the events. Call once per control tick.
Commands takeCommands();

// Hand the latest snapshot to the publisher task.
void publish(const Telemetry& t);

// Acknowledge a command applied by the loop. `error` is sent only when ok is false.
void ack(uint32_t id, bool ok, const char* error = nullptr);

}  // namespace wifi_link
