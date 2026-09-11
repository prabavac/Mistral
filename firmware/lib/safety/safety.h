// safety.h — arming gates and saturation flags — Mistral
//
// The actuator clamps themselves live where the pulses are generated (lib/tvc,
// lib/motors). This module gathers their saturation flags and owns the arming
// gates.
//
// Saturation flags propagate BACK UP the control chain: when an actuator clamps,
// every integrator upstream of it freezes instead of winding up against a limit it
// cannot exceed (the tvc-drone reference's anti-windup approach). Consumers:
// control/lqr now, and any future velocity/position loops.
#pragma once

#include <motors.h>
#include <tvc.h>

namespace safety {

struct Saturation {
    bool tvcX;    // servo X hit TVC_CLAMP_DEG or its µs backstop
    bool tvcY;    // servo Y hit TVC_CLAMP_DEG or its µs backstop
    bool thrust;  // a motor command was clipped to the ESC range
};

// Collect this tick's actuator saturation flags.
Saturation saturation(const tvc::Status& t, const motors::Status& m);

// Initial gate set — extended when the state machine is implemented.
struct ArmInputs {
    bool  imuValid;
    bool  attitudeValid;
    float commonThrust;  // commanded collective; must be zero to arm
};

// True only when every arming gate passes.
bool armingAllowed(const ArmInputs& in);

}  // namespace safety
