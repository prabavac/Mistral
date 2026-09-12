// state_machine.h — flight state — Mistral
//
//   DISARMED → ARMED → FLYING → LANDED
//   any state → KILL
//
// DISARMED: both ESCs held at ESC_MIN_US, control loops BYPASSED, and every
//           controller integrator held at zero, so nothing winds up while the
//           vehicle sits on the ground.
// ARMED:    reached only through safety::armingAllowed() and throttle::arm().
// KILL:     both ESCs forced to ESC_MIN_US immediately.
#pragma once

#include <cstdint>

namespace state_machine {

enum class State : uint8_t { DISARMED, ARMED, FLYING, LANDED, KILL };

// Enter DISARMED. Call once in setup(), after throttle::init().
void init();

// Advance transitions. Call once per control tick.
void update();

State state();

// False in DISARMED and KILL: attitude controllers must not run, and their
// integrators stay at zero.
bool controlActive();

// Returns false if an arming gate refuses.
bool requestArm();
void requestDisarm();
void kill();

}  // namespace state_machine
