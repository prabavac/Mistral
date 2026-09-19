// state_machine.h: flight state for Mistral
//
//   DISARMED ⇄ ARMED ⇄ FLYING          DISARM, KILL, link loss: any state → DISARMED
//
// DISARMED: controller bypassed, integrals zero, servos centred, ESCs at minimum.
// ARMED:    PD live on the servos (the LQI with its integrals forced to zero), ESCs at
//           minimum. Reached from DISARMED only when the arming gates pass
//           (safety::armingAllowed), or from FLYING to stop the motors. This is the state
//           for hand-tilt sign checks.
// FLYING:   ESCs armed through throttle::arm()'s hold, throttle live. Integrals accumulate
//           only at or above cfg::LQR_INTEGRATE_MIN_THROTTLE: below it the vehicle is on the
//           pad, the loop is open, and they would wind up before lift-off. If the ESCs drop
//           out on their own (auto-cut, or the arm was refused), falls back to ARMED.
#pragma once

#include <cstdint>

#include <safety.h>

namespace state_machine {

enum class State : uint8_t { DISARMED, ARMED, FLYING };

// Enter DISARMED. Call once in setup(), after throttle::init().
void init();

// ARM: DISARMED → ARMED if the gates pass; FLYING → ARMED (motors off). False if refused.
bool requestArm(const safety::ArmInputs& in);

// FLY: ARMED → FLYING, starting the ESC arming hold. False unless ARMED and the ESCs accepted.
bool requestFly();

// DISARM, KILL, link loss: → DISARMED, ESCs to minimum immediately. Safe to call any time.
void disarm();

// Follow the ESCs out of FLYING. Call once per control tick, after throttle::update().
void update();

State       state();
const char* name(State s);

bool controlActive();  // ARMED or FLYING
bool integrating();    // FLYING, ESCs armed, throttle ≥ cfg::LQR_INTEGRATE_MIN_THROTTLE

}  // namespace state_machine
