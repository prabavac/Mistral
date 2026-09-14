// lqr.h — attitude LQI — Mistral (lib/control)
//
// 6-state LQI. EVERYTHING IN RADIANS: x = [r1, r2, dr1, dr2, ir1, ir2], u = nozzle rad.
//   u = −K·x · cfg::LQR_GAIN_SCALE · thrustScale, clamped to ±cfg::TVC_CLAMP_RAD
// K is symmetric; all mechanical sign lives in ServoCal::dir (see cfg::LQR_K). x_ref = 0:
// hold the pose captured at boot.
//
// Integrator gate: ir = ∫r dt accumulates ONLY when `integrate` is true
// (state_machine::integrating — FLYING at or above the throttle threshold) and is
// forced to zero otherwise. On the bench the loop is open — the servos move but the
// vehicle doesn't rotate — so an ungated integrator winds to its clamp.
//
// Anti-windup (the bench sketch's rule): block only an increment that would push an axis
// further into saturation — its u past the clamp, or its servo on a µs backstop. A blanket
// freeze traps a wound-up integrator with no way back.
//
// Thrust scheduling (cfg::THRUST_SCHED, off): thrustScale = NOMINAL / max(thrust, LCLAMP).
// Applied before the clamp, as in the bench sketch, so the nozzle limit still holds.
#pragma once

#include <attitude.h>
#include <safety.h>

namespace lqr {

struct Output {
    float u1, u2;        // rad — clamped nozzle commands for servo X / Y
    float uRaw1, uRaw2;  // rad — before the clamp
    float ir1, ir2;      // rad·s — integral states after this step
};

// Zero the integrals.
void reset();

// One controller step; dt_s in seconds. `sat` is the previous command's actuator
// saturation. thrust_N feeds scheduling and is ignored while cfg::THRUST_SCHED is off.
Output update(const attitude::Estimate& est, bool integrate, const safety::Saturation& sat,
              float thrust_N, float dt_s);

}  // namespace lqr
