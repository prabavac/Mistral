// lqr.h — attitude LQR — Mistral (lib/control)
//
// The attitude controller. u = −K·x with K = cfg::LQR_K (UNTUNED placeholder,
// computed offline from the vehicle model). State and input ordering are
// documented next to cfg::LQR_K.
//
// Runs only while state_machine::controlActive(). In DISARMED it is bypassed and
// reset() holds the integral states at zero.
//
// Anti-windup: integral states on saturated axes (safety::Saturation) freeze.
//
// Which of TVC X/Y corrects pitch vs roll depends on the gimbal mount — verify on
// the stand.
#pragma once

#include <attitude.h>
#include <safety.h>

namespace lqr {

struct Setpoint {
    float pitch_deg;
    float roll_deg;
    float yawRate_dps;  // yaw is rate only — no heading setpoint
};

// Actuator demand, in the units lib/tvc and lib/throttle accept.
struct Command {
    float tvcX_deg;
    float tvcY_deg;
    float differential;  // yaw — passed to throttle::setDifferential()
};

// Zero the integral states. Called on entry to DISARMED.
void reset();

// One controller step. Call once per control tick; dt_s in seconds.
Command update(const Setpoint& sp, const attitude::Estimate& est,
               const safety::Saturation& sat, float dt_s);

}  // namespace lqr
