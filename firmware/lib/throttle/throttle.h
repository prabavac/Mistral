// throttle.h — coaxial contra-rotating motor ESCs — Mistral
//
// TWO INDEPENDENT ESC channels: ESC1 = upper motor, ESC2 = lower motor. Never Y-split
// the signal — differential thrust is the yaw actuator, so each motor must be
// separately commandable. Each ESC has its own pin, LEDC channel and write; there is
// no shared "write all PWM" loop and no write path shared with lib/tvc.
//
// Software arming gate, kept as defence in depth:
//   - both ESCs are forced to cfg::ESC_MIN_US unless ARMED;
//   - arm() holds ESC_MIN for cfg::ARM_HOLD_MS (ARMING) before throttle is accepted;
//   - arm() is refused while the commanded throttle is non-zero, and moving the
//     throttle off zero during the hold aborts arming, so arming can never jump
//     straight to a stale throttle value;
//   - auto-cut: disarm when throttle is non-zero and unchanged for
//     cfg::THROTTLE_AUTOCUT_MS.
//
// No voltage compensation yet: this build has no voltage sense.
//
// PWM — native LEDC only. init() calls ledcAttachChannel(pin, 50, 14, ch) ONCE per
// ESC, on the explicit channels cfg::ESC1_LEDC_CH / ESC2_LEDC_CH. ledcWrite() takes
// the PIN (core 3.x). usToDuty = (us << 14) / 20000. Never attach in loop(), never
// use ledcAttach auto-channel, never ESP32Servo (its MCPWM timers caused uncommanded
// ESC spin on Zephyr).
#pragma once

#include <cstdint>

namespace throttle {

enum class State : uint8_t { DISARMED, ARMING, ARMED };

struct Status {
    State    state;
    uint32_t armRemainingMs;    // ARMING: time left in the ESC_MIN hold; otherwise 0
    float    normalised;        // commanded throttle 0..1, held even while disarmed
    float    differential;      // commanded yaw differential
    uint16_t esc1_us, esc2_us;  // last pulse commanded to each ESC
    bool     saturated;         // a motor command was clipped to the ESC range
};

// Attach both ESC channels once and hold them at ESC_MIN_US. Returns false if either
// attach fails — no pulses are produced then, so the caller must halt. Call FIRST in
// setup().
bool init();

// Start arming: DISARMED → ARMING → ARMED after cfg::ARM_HOLD_MS. Returns false, and
// stays DISARMED, if the commanded throttle is non-zero. Idempotent.
bool arm();

// Both ESCs to ESC_MIN_US immediately. Safe to call any time. The commanded throttle
// is kept, so arm() stays refused until it returns to zero.
void disarm();

// Commanded throttle, 0..1 (clamped; NaN reads as 0). Drives the motors only while
// ARMED.
void setNormalised(float value);

// Yaw: +delta on ESC1, −delta on ESC2, around the common throttle. Sign: VERIFY on the
// stand. Each motor is clipped to 0..1, which sets saturated. At zero throttle both
// motors stay at min whatever the differential.
void setDifferential(float delta);

// Advance arming and auto-cut, then write both ESCs. Call once per control tick.
void update();

// BENCH ONLY — PROPS OFF. Blocking ESC throttle-range calibration (~12 s): MIN while
// the battery is unplugged, MAX while it is plugged back in, then MIN for the low
// point. Drives MAX regardless of the arming gate. Refused unless DISARMED. Progress
// is printed to Serial.
bool calibrate();

Status status();

}  // namespace throttle
