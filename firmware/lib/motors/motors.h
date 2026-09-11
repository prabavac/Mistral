// motors.h — coaxial contra-rotating motor ESCs — Mistral
//
// TWO INDEPENDENT ESC channels. Never Y-split the signal: yaw is differential
// thrust, so each motor must be separately commandable. Each ESC has its own pin,
// its own LEDC channel and its own write — no shared "write all PWM" loop, and no
// write path shared with lib/tvc.
//
// Throttle is software arming-gated: unless armed(), both ESCs are forced to
// cfg::ESC_MIN_US no matter what setThrust() asks for.
//
// PWM — native LEDC only. init() calls ledcAttachChannel(pin, 50, 14, ch) ONCE
// per ESC, on the explicit channels cfg::ESC1_LEDC_CH / ESC2_LEDC_CH, then writes
// cfg::ESC_MIN_US. ledcWrite() takes the PIN (core 3.x).
// usToDuty = (us << 14) / 20000. Never attach in loop(), never use ledcAttach
// auto-channel, never ESP32Servo (its MCPWM timers caused uncommanded ESC spin on
// Zephyr).
#pragma once

#include <cstdint>

namespace motors {

struct Status {
    uint16_t esc1_us, esc2_us;  // last pulse written to each ESC
    bool     armed;
    bool     saturated;         // a motor command was clipped to the ESC range
};

// Attach both ESC channels once and hold them at ESC_MIN_US. Call FIRST in setup().
void init();

// Start arming: both ESCs held at ESC_MIN_US for cfg::ARM_HOLD_MS before thrust is
// accepted. Called only by state_machine, after safety::armingAllowed().
void arm();

// Both ESCs to ESC_MIN_US immediately. Safe to call any time.
void disarm();

bool armed();

// common:       collective thrust, 0..1.
// differential: yaw command, added to one motor and subtracted from the other.
//               Which motor takes the + side is verified on the stand.
// Each motor's result is clipped to the ESC range; clipping sets Status::saturated.
void setThrust(float common, float differential);

Status status();

}  // namespace motors
