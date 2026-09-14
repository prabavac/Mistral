#include "state_machine.h"

#include <mistral_config.h>
#include <throttle.h>

namespace {

state_machine::State current = state_machine::State::DISARMED;

}  // namespace

void state_machine::init() { disarm(); }

bool state_machine::requestArm(const safety::ArmInputs& in) {
    switch (current) {
        case State::FLYING:
            throttle::disarm();
            current = State::ARMED;
            return true;
        case State::ARMED:
            return true;
        case State::DISARMED:
            if (!safety::armingAllowed(in)) return false;
            current = State::ARMED;
            return true;
    }
    return false;
}

bool state_machine::requestFly() {
    if (current == State::FLYING) return true;
    if (current != State::ARMED || !throttle::arm()) return false;
    current = State::FLYING;
    return true;
}

void state_machine::disarm() {
    throttle::disarm();
    current = State::DISARMED;
}

void state_machine::update() {
    if (current == State::FLYING && throttle::status().state == throttle::State::DISARMED) {
        current = State::ARMED;
    }
}

state_machine::State state_machine::state() { return current; }

const char* state_machine::name(State s) {
    switch (s) {
        case State::ARMED:  return "ARMED";
        case State::FLYING: return "FLYING";
        default:            return "DISARMED";
    }
}

bool state_machine::controlActive() { return current != State::DISARMED; }

bool state_machine::integrating() {
    const throttle::Status t = throttle::status();
    return current == State::FLYING && t.state == throttle::State::ARMED &&
           t.normalised >= cfg::LQR_INTEGRATE_MIN_THROTTLE;
}
