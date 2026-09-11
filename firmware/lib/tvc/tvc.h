// tvc.h — thrust-vector gimbal actuator — Mistral
//
// Two gear-reduced MG90S servos (X and Y). The interface speaks TVC DEGREES —
// deflection at the gimbal, not at the servo horn — and this module applies each
// servo's gear ratio internally. Per servo, from its cfg::ServoCal:
//   1. clamp to ±cfg::TVC_CLAMP_DEG               PRIMARY limit, EVERY servo
//   2. servoDeg = tvcDeg · gearRatio · dir
//   3. us = centerUs + servoDeg · usPerServoDeg
//   4. clamp us to [minUs, maxUs]                 hard µs backstop
//   5. write only if us changed                   re-issuing the same pulse
//                                                 makes the MG90S buzz
// Either clamp engaging sets that servo's saturation flag in Status.
//
// PWM — native LEDC only. init() calls ledcAttachChannel(pin, 50, 14, ch) ONCE
// per servo, on the explicit channels cfg::SERVO_X_LEDC_CH / SERVO_Y_LEDC_CH.
// ledcWrite() takes the PIN (core 3.x). usToDuty = (us << 14) / 20000.
// Never attach in loop(), never use ledcAttach auto-channel, never ESP32Servo.
//
// Every servo sign (ServoCal::dir) must be verified on the stand before free flight.
#pragma once

#include <cstdint>

namespace tvc {

struct Status {
    float    x_deg, y_deg;  // last commanded TVC deflection, after the degree clamp
    uint16_t x_us, y_us;    // last pulse written
    bool     x_saturated;   // degree clamp or µs backstop engaged on the last command
    bool     y_saturated;
};

// Attach both servo LEDC channels once, then center(). Call once in setup().
void init();

// Command 0° on both axes (each servo to its own centerUs).
void center();

// Command TVC deflection in degrees at the gimbal.
void setDeflection(float x_deg, float y_deg);

Status status();

}  // namespace tvc
