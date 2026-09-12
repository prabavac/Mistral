// mistral_config.h — Mistral thrust-vectored coaxial drone
//
// SINGLE SOURCE OF TRUTH for every hardware constant: pins, LEDC channels, I2C
// addresses, PWM limits, servo calibration, gains and clamps. Nothing magic is
// buried in module logic — if a number is tied to the airframe, the electronics
// or tuning, it lives here.
//
// Markers:
//   MEASURED    came off the bench — authoritative
//   UNVERIFIED  inherited from the reference or a datasheet — confirm on the stand
//   UNTUNED     placeholder gain — do not fly on it
//   VERIFY      a sign/direction — confirm on the stand before free flight
//   TBD         not chosen yet
//
// Board: Heltec WiFi LoRa 32 V3 (ESP32-S3), Arduino-ESP32 core 3.x.

#pragma once

#include <cstdint>

namespace cfg {

// ─────────────────────────────── Serial ─────────────────────────────────────
constexpr uint32_t SERIAL_BAUD = 115200;  // matches monitor_speed in platformio.ini

// ─────────────────────────── I2C sensor bus ─────────────────────────────────
constexpr uint8_t I2C_SDA = 41;
constexpr uint8_t I2C_SCL = 42;

constexpr uint8_t ICM42688_I2C_ADDR = 0x68;  // bring-up only; SPI preferred for flight
constexpr uint8_t BMP388_I2C_ADDR   = 0x76;  // 0x77 if SDO is pulled high

// ─────────────────────── On-board OLED (SSD1306) ────────────────────────────
constexpr uint8_t OLED_SDA  = 17;
constexpr uint8_t OLED_SCL  = 18;
constexpr uint8_t OLED_RST  = 21;
constexpr uint8_t VEXT      = 36;  // OLED power enable, active LOW
constexpr uint8_t OLED_ADDR = 0x3C;

// ─────────────────── MTF-01P optical flow + range (UART) ────────────────────
constexpr uint8_t  PIN_UNASSIGNED   = 0xFF;
constexpr uint8_t  MTF01_RX_PIN     = PIN_UNASSIGNED;  // TBD — ESP32 RX ← sensor TX
constexpr uint8_t  MTF01_TX_PIN     = PIN_UNASSIGNED;  // TBD — ESP32 TX → sensor RX
constexpr uint32_t MTF01_BAUD       = 115200;
constexpr uint32_t MTF01_TIMEOUT_MS = 100;  // TBD — set from the measured frame rate

// ─────────────────────────── LEDC PWM (50 Hz) ───────────────────────────────
// ESCs and servos all run on native LEDC: ledcAttachChannel(pin, PWM_FREQ_HZ,
// PWM_RES_BITS, ch) on the EXPLICIT channels below, ONCE, in the owning module's
// init(). µs → duty: (us << PWM_RES_BITS) / PWM_PERIOD_US.
// PWM_RES_BITS must stay 14: the ESP32-S3 LEDC timer is at most 14 bits, and a wider
// value makes ledcAttachChannel() fail — no pulses at all. 14 bits at 50 Hz is
// 1.22 µs per count, finer than the MG90S's 5 µs dead band.
constexpr uint32_t PWM_FREQ_HZ   = 50;
constexpr uint8_t  PWM_RES_BITS  = 14;
constexpr uint32_t PWM_PERIOD_US = 20000;

// Each ESC signal pin needs a 10 kΩ pulldown to ground — GPIOs float during boot.
constexpr uint8_t ESC1_PIN     = 6;  // upper motor
constexpr uint8_t ESC2_PIN     = 7;  // lower motor
constexpr uint8_t ESC1_LEDC_CH = 0;
constexpr uint8_t ESC2_LEDC_CH = 1;

constexpr uint8_t SERVO_X_PIN     = 48;  // pitch
constexpr uint8_t SERVO_Y_PIN     = 5;   // roll — not GPIO 4 (confirmed wiring fault on Zephyr)
constexpr uint8_t SERVO_X_LEDC_CH = 2;
constexpr uint8_t SERVO_Y_LEDC_CH = 3;

// ─────────────────────────────── ESCs ───────────────────────────────────────
constexpr uint16_t ESC_MIN_US  = 1000;  // disarmed / zero thrust
constexpr uint16_t ESC_MAX_US  = 2000;
constexpr uint32_t ARM_HOLD_MS = 3000;  // ESC_MIN hold before throttle is accepted

// Auto-cut: disarm when throttle is non-zero and unchanged for this long, so an
// unattended bench rig can't run the pack flat.
constexpr uint32_t THROTTLE_AUTOCUT_MS = 60000;  // TBD

// throttle::calibrate() timing — MEASURED working values. After MAX, the ESC accepts
// the low point only within ~4 s of battery connect; miss it and it enters its
// programming menu instead.
constexpr uint32_t ESC_CAL_UNPLUG_MS = 5000;  // at MIN: unplug the battery
constexpr uint32_t ESC_CAL_MAX_MS    = 3000;  // at MAX: plug the battery in
constexpr uint32_t ESC_CAL_MIN_MS    = 4000;  // at MIN: low point, then self-detect tone

// ──────────────────────────── Control loop ──────────────────────────────────
constexpr uint16_t CONTROL_LOOP_HZ = 100;

// ──────────────────────────────── TVC ───────────────────────────────────────
// Deflection clamp in TVC degrees — PRIMARY limit, enforced in lib/tvc for EVERY
// servo. Clamping in degrees rather than µs gives both axes the same angular limit
// even though their gear ratios differ. Bring-up value.
constexpr float TVC_CLAMP_DEG = 10.0f;

// Gear reduction: servo degrees per TVC degree. UNVERIFIED — inherited from the
// reference. A ±500 µs sweep on this gimbal moves Y about a quarter less than X,
// consistent with 3:1 vs 4:1, but that only confirms the ratio BETWEEN the axes.
// Measure absolute deflection with a protractor before flight: this scales every
// commanded correction.
constexpr float GEAR_RATIO_X = 3.0f;
// GEAR_RATIO_Y is 4.0, NOT 3.0 — deliberate. Do not "fix" it to match X.
// Servo-y's gear revolves around the TVC axis as the gimbal rotates, so the servo
// needs one extra full rotation per 360° of TVC travel: 3 + 1 = 4.
constexpr float GEAR_RATIO_Y = 4.0f;

// Per-servo calibration. lib/tvc maps a clamped TVC angle to a pulse:
//   us = centreUs + trim + tvcDeg · gearRatio · dir · usPerServoDeg
// then clamps us to [minUs, maxUs] — the hard backstop behind TVC_CLAMP_DEG.
struct ServoCal {
    uint16_t centreUs;       // pulse at 0° TVC
    float    usPerServoDeg;  // µs per degree at the servo horn, before gearing
    float    gearRatio;      // servo degrees per TVC degree
    int8_t   dir;            // +1 or -1
    uint16_t minUs;          // hard backstop
    uint16_t maxUs;          // hard backstop
};

// MEASURED: centres, and ±SERVO_SPAN_US about each centre is clear of the mechanical
// stops (X 1126–2126, Y 1084–2084). The backstops are exactly that verified range —
// a stalled MG90S draws ~700 mA and strips teeth, so never widen past what was checked.
constexpr uint16_t SERVO_X_CENTRE_US = 1626;
constexpr uint16_t SERVO_Y_CENTRE_US = 1584;
constexpr uint16_t SERVO_SPAN_US     = 500;

// MEASURED absolute pulse limits for either servo. Every per-servo backstop must sit
// inside them — checked at compile time below.
constexpr uint16_t SERVO_HARD_MIN_US = 1000;
constexpr uint16_t SERVO_HARD_MAX_US = 2250;

// usPerServoDeg 10.5 is UNVERIFIED until the protractor check. dir: VERIFY on the stand.
// At the bring-up clamp X needs ±315 µs and Y ±420 µs — inside the ±500 backstops.
constexpr ServoCal SERVO_X_CAL = {SERVO_X_CENTRE_US, 10.5f, GEAR_RATIO_X, +1,
                                  SERVO_X_CENTRE_US - SERVO_SPAN_US,
                                  SERVO_X_CENTRE_US + SERVO_SPAN_US};
constexpr ServoCal SERVO_Y_CAL = {SERVO_Y_CENTRE_US, 10.5f, GEAR_RATIO_Y, +1,
                                  SERVO_Y_CENTRE_US - SERVO_SPAN_US,
                                  SERVO_Y_CENTRE_US + SERVO_SPAN_US};

static_assert(SERVO_X_CAL.minUs >= SERVO_HARD_MIN_US && SERVO_X_CAL.maxUs <= SERVO_HARD_MAX_US,
              "servo X backstop outside SERVO_HARD_MIN_US..SERVO_HARD_MAX_US");
static_assert(SERVO_Y_CAL.minUs >= SERVO_HARD_MIN_US && SERVO_Y_CAL.maxUs <= SERVO_HARD_MAX_US,
              "servo Y backstop outside SERVO_HARD_MIN_US..SERVO_HARD_MAX_US");

// ──────────────────────── Translation estimate ──────────────────────────────
// Low-pass on the range term, which scales the whole velocity estimate:
//   y = α·y + (1 − α)·x
constexpr float RANGE_LPF_ALPHA = 0.9f;  // UNTUNED

// ─────────────────── Attitude LQR — UNTUNED PLACEHOLDERS ────────────────────
// u = −K·x, with K computed offline from the vehicle model. All zero: not computed.
//   x = [pitch, roll, pitchRate, rollRate, yawRate, ∫pitch, ∫roll, ∫yawRate]
//   u = [tvcX_deg, tvcY_deg, differential]
constexpr uint8_t LQR_STATES = 8;
constexpr uint8_t LQR_INPUTS = 3;
constexpr float   LQR_K[LQR_INPUTS][LQR_STATES] = {};  // UNTUNED

// ──────────────────── WiFi SoftAP + WebSocket (lib/wifi_link) ───────────────
// The vehicle is the access point. SSID = prefix + last 4 hex digits of the AP MAC.
// Page at http://192.168.4.1, WebSocket at WIFI_WS_PATH.
constexpr char     WIFI_SSID_PREFIX[] = "mistral-";
constexpr char     WIFI_PASSWORD[]    = "mistral-flight";  // WPA2, 8+ characters
constexpr uint16_t WIFI_HTTP_PORT     = 80;
constexpr char     WIFI_WS_PATH[]     = "/ws";
// Network work runs on core 0; the control loop runs on core 1. Must match
// -D CONFIG_ASYNC_TCP_RUNNING_CORE in platformio.ini.
constexpr uint8_t  WIFI_CORE    = 0;
constexpr uint16_t TELEMETRY_HZ = 10;  // push rate — not the 100 Hz loop rate
// Link loss: the ground page pings every 250 ms. With no frame from any client for this
// long, loop() disarms, so a dropped phone or a locked screen can't leave motors running.
constexpr uint32_t LINK_TIMEOUT_MS = 1000;

// ──────────────────────── Bench tests (lib/bench) ───────────────────────────
// Bring-up only. While true, loop() sweeps the gimbal diagonally: both axes to
// +TVC_CLAMP_DEG, hold, then both to −TVC_CLAMP_DEG, hold. Set false once a controller
// drives the servos.
constexpr bool     BENCH_SERVO_SWEEP    = false;  // off: servos held at centre (tvc::init)
constexpr uint32_t BENCH_SWEEP_DWELL_MS = 1500;  // hold at each end

}  // namespace cfg
