// lqr.h: attitude LQI for Mistral (lib/control)
//
// 6-state LQI. EVERYTHING IN RADIANS: x = [r1, r2, dr1, dr2, ir1, ir2], u = nozzle rad.
//   u_i = −(kth·r_i + kq·dr_i + ki·ir_i) · scale · thrustScale, clamped to ±cfg::TVC_CLAMP_RAD
// The gains start from the solved cfg::LQR_K (symmetric; all mechanical sign lives in
// ServoCal::dir) and can be changed live from the ground station. setGains() clamps every value
// to the ranges in mistral_config.h, whatever the sender asked for. x_ref = 0: hold the boot pose.
//
// Integral action is OFF at boot (cfg::LQR_KI_BOOT_FACTOR = 0). An axis whose ki is 0 holds its
// integral at zero, so raising ki later starts from zero instead of releasing a stored value.
//
// Integrator gate: ir = ∫r dt accumulates ONLY when `integrate` is true
// (state_machine::integrating, FLYING at or above the throttle threshold) and is forced to zero
// otherwise. On the bench the loop is open (the servos move but the vehicle doesn't rotate), so
// an ungated integrator winds to its clamp.
//
// Anti-windup (the bench sketch's rule): block only an increment that would push an axis further
// into saturation: its u past the clamp, or its servo on a µs backstop. A blanket freeze traps a
// wound-up integrator with no way back.
//
// Thrust scheduling (cfg::THRUST_SCHED, off): thrustScale = NOMINAL / max(thrust, LCLAMP).
// Applied before the clamp, as in the bench sketch, so the nozzle limit still holds.
#pragma once

#include <cstdint>

#include <attitude.h>
#include <safety.h>

namespace lqr {

struct AxisGains {
    float kth;  // angle term, per rad
    float kq;   // rate term, per rad/s
    float ki;   // integral term, per rad·s
};

struct Gains {
    float     scale;  // multiplies every term
    AxisGains x;      // servo X, corrects r1
    AxisGains y;      // servo Y, corrects r2
};

struct Output {
    float u1, u2;        // rad, clamped nozzle commands for servo X / Y
    float uRaw1, uRaw2;  // rad, before the clamp
    float ir1, ir2;      // rad·s, integral states after this step
};

// The solved gains for one axis (0 = X, 1 = Y), straight from cfg::LQR_K.
AxisGains designGains(uint8_t axis);

// What the controller boots with: the solved gains, scale cfg::LQR_GAIN_SCALE, integral term
// × cfg::LQR_KI_BOOT_FACTOR.
Gains bootGains();

// The gains in use now.
Gains gains();

// Replace the live gains. Each value is clamped to its cfg range (NaN keeps the current value).
// Returns what was applied.
Gains setGains(const Gains& g);

// Zero the integrals.
void reset();

// One controller step; dt_s in seconds. `sat` is the previous command's actuator saturation.
// thrust_N feeds scheduling and is ignored while cfg::THRUST_SCHED is off.
Output update(const attitude::Estimate& est, bool integrate, const safety::Saturation& sat,
              float thrust_N, float dt_s);

}  // namespace lqr
