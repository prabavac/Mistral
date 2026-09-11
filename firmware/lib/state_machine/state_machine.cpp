#include "state_machine.h"

// STUB — implemented later. Reports DISARMED and refuses to arm until then.

void state_machine::init() {}

void state_machine::update() {}

state_machine::State state_machine::state() { return State::DISARMED; }

bool state_machine::controlActive() { return false; }

bool state_machine::requestArm() { return false; }

void state_machine::requestDisarm() {}

void state_machine::kill() {}
