// mistral_config.h — Mistral thrust-vectored coaxial drone
//
// SINGLE SOURCE OF TRUTH for every hardware constant: pins, LEDC channels, I2C
// addresses, PWM limits, servo calibration, gains and clamps. Nothing magic is
// buried in module logic — if a number is tied to the airframe, the electronics
// or tuning, it lives here.
//
// Markers:
//   UNTUNED    placeholder gain — do not fly on it
//   CALIBRATE  depends on the physical build — measure it
//   VERIFY     a sign/direction — confirm on the stand before free flight
//   TBD        not chosen yet
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
constexpr uint32_t PWM_FREQ_HZ   = 50;
constexpr uint8_t  PWM_RES_BITS  = 14;
constexpr uint32_t PWM_PERIOD_US = 20000;

constexpr uint8_t ESC1_PIN     = 6;
constexpr uint8_t ESC2_PIN     = 7;
constexpr uint8_t ESC1_LEDC_CH = 0;
constexpr uint8_t ESC2_LEDC_CH = 1;

constexpr uint8_t SERVO_X_PIN     = 48;
constexpr uint8_t SERVO_Y_PIN     = 5;
constexpr uint8_t SERVO_X_LEDC_CH = 2;
constexpr uint8_t SERVO_Y_LEDC_CH = 3;

// ─────────────────────────────── ESCs ───────────────────────────────────────
constexpr uint16_t ESC_MIN_US  = 1000;  // disarmed / zero thrust
constexpr uint16_t ESC_MAX_US  = 2000;
constexpr uint32_t ARM_HOLD_MS = 2000;  // CALIBRATE — ESC_MIN hold before thrust is accepted (Zephyr's value)

// ──────────────────────────── Control loop ──────────────────────────────────
constexpr uint16_t CONTROL_LOOP_HZ = 100;

// ──────────────────────────────── TVC ───────────────────────────────────────
// Deflection clamp in TVC degrees — PRIMARY limit, enforced in lib/tvc for EVERY
// servo. Bring-up value.
constexpr float TVC_CLAMP_DEG = 10.0f;

// Gear reduction: servo degrees per TVC degree.
constexpr float GEAR_RATIO_X = 3.0f;
// GEAR_RATIO_Y is 4.0, NOT 3.0 — deliberate. Do not "fix" it to match X.
// Servo-y's gear revolves around the TVC axis as the gimbal rotates, so the servo
// needs one extra full rotation per 360° of TVC travel: 3 + 1 = 4.
constexpr float GEAR_RATIO_Y = 4.0f;

// Per-servo calibration. lib/tvc maps a clamped TVC angle to a pulse:
//   us = centerUs + tvcDeg · gearRatio · dir · usPerServoDeg
// then clamps us to [minUs, maxUs] — the hard backstop behind TVC_CLAMP_DEG.
struct ServoCal {
    uint16_t centerUs;       // CALIBRATE — pulse at 0° TVC
    float    usPerServoDeg;  // CALIBRATE — µs per degree at the servo horn, before gearing
    float    gearRatio;      // servo degrees per TVC degree
    int8_t   dir;            // VERIFY — +1 or -1
    uint16_t minUs;          // CALIBRATE — hard backstop, bench-measured gimbal limit
    uint16_t maxUs;          // CALIBRATE — hard backstop, bench-measured gimbal limit
};

// Placeholders. 11.1 µs/deg is a nominal MG90S figure (1000 µs per 90°). At the
// bring-up clamp X spans ±333 µs and Y ±444 µs around center, inside the backstops.
constexpr ServoCal SERVO_X_CAL = {1500, 11.1f, GEAR_RATIO_X, +1, 1000, 2000};
constexpr ServoCal SERVO_Y_CAL = {1500, 11.1f, GEAR_RATIO_Y, +1, 1000, 2000};

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

}  // namespace cfg
