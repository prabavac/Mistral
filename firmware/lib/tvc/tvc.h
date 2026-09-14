// tvc.h — thrust-vector gimbal servos — Mistral
//
// Two gear-reduced MG90S servos: X corrects r1 (rotation about body X), Y corrects r2.
// The interface speaks NOZZLE RADIANS — deflection at the nozzle, not the servo horn — and
// this module is the mixer: the only place the gear ratios and degrees appear. Per servo,
// from its cfg::ServoCal:
//   1. clamp to ±cfg::TVC_CLAMP_RAD, in nozzle radians   PRIMARY limit, EVERY servo
//   2. us = centreUs + trim + deg(nozzle) · gearRatio · dir · usPerServoDeg
//   3. clamp us to [minUs, maxUs]                          hard µs backstop
//   4. write at most once per 50 Hz PWM frame, and only when the target has moved at
//      least cfg::SERVO_DEADBAND_US from the last pulse written
// Either clamp engaging sets that servo's saturation flag.
//
// NO software interpolation. The target pulse is written directly and the servo slews to
// it natively; stepping through intermediate targets is visibly steppy.
//
// PWM — native LEDC only. init() calls ledcAttachChannel(pin, 50, 14, ch) ONCE per servo,
// on the explicit channels cfg::SERVO_X_LEDC_CH / SERVO_Y_LEDC_CH. ledcWrite() takes the
// PIN (core 3.x). Never attach in loop(), never use ledcAttach auto-channel, never
// ESP32Servo.
//
// Every servo sign (ServoCal::dir) must be verified on the stand before free flight.
#pragma once

#include <cstdint>

namespace tvc {

enum class Axis : uint8_t { X, Y };

struct Status {
    float    x_rad, y_rad;          // last commanded nozzle deflection, after the clamp
    uint16_t x_us, y_us;            // target pulse for that deflection
    int16_t  x_trim_us, y_trim_us;  // live centre trim
    bool     x_saturated;           // nozzle clamp or µs backstop engaged on the last command
    bool     y_saturated;
};

// Attach both servo channels once and park at centre. Returns false if either attach
// fails — no pulses are produced then, so the caller must halt.
bool init();

// Command 0 on both axes.
void centre();

// Command nozzle deflection in radians: X corrects r1, Y corrects r2. Call every control
// tick — writes happen on the 50 Hz frame.
void setDeflection(float xRad, float yRad);

// Live centre adjustment in µs, added on top of the servo's centreUs. Re-applies the last
// commanded deflection; the µs backstops still apply.
void setTrim(Axis axis, int16_t us);

Status status();

}  // namespace tvc
