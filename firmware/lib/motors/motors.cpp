#include "motors.h"

// STUB — implemented during bring-up, against real hardware. ESC pins stay
// unattached (no pulses) until this is written.

void motors::init() {}

void motors::arm() {}

void motors::disarm() {}

bool motors::armed() { return false; }

void motors::setThrust(float, float) {}

motors::Status motors::status() { return {}; }
