// tvc.h — thrust-vector gimbal servos — Mistral
//
// Two gear-reduced MG90S servos: X = pitch, Y = roll. The interface speaks TVC
// DEGREES — deflection at the nozzle, not at the servo horn — and applies each
// servo's gear ratio internally. Per servo, from its cfg::ServoCal:
//   1. clamp to ±cfg::TVC_CLAMP_DEG, in TVC degrees     PRIMARY limit, EVERY servo
//   2. us = centreUs + trim + tvcDeg · gearRatio · dir · usPerServoDeg
//   3. clamp us to [minUs, maxUs]                       hard µs backstop
//   4. write only if the duty count changed             re-issuing buzzes the MG90S
// Either clamp engaging sets that servo's saturation flag.
//
// NO software interpolation. The target pulse is written directly and the servo
// slews to it natively; stepping through intermediate targets is visibly steppy.
//
// PWM — native LEDC only. init() calls ledcAttachChannel(pin, 50, 14, ch) ONCE per
// servo, on the explicit channels cfg::SERVO_X_LEDC_CH / SERVO_Y_LEDC_CH.
// ledcWrite() takes the PIN (core 3.x). usToDuty = (us << 14) / 20000.
// Never attach in loop(), never use ledcAttach auto-channel, never ESP32Servo.
//
// Every servo sign (ServoCal::dir) must be verified on the stand before free flight.
#pragma once

#include <cstdint>

namespace tvc {

enum class Axis : uint8_t { X, Y };

struct Status {
    float    x_deg, y_deg;          // last commanded TVC deflection, after the degree clamp
    uint16_t x_us, y_us;            // last pulse commanded
    int16_t  x_trim_us, y_trim_us;  // live centre trim
    bool     x_saturated;           // degree clamp or µs backstop engaged on the last command
    bool     y_saturated;
};

// Attach both servo channels once and park at centre. Returns false if either attach
// fails — no pulses are produced then, so the caller must halt.
bool init();

// Command 0° on both axes.
void centre();

// Command TVC deflection in degrees at the nozzle: pitch on X, roll on Y.
void setDeflection(float pitchDeg, float rollDeg);

// Live centre adjustment in µs, added on top of the servo's centreUs. Re-applies the
// last commanded deflection; the µs backstops still apply.
void setTrim(Axis axis, int16_t us);

Status status();

}  // namespace tvc
