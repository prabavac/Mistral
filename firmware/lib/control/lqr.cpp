#include "lqr.h"

// STUB — implemented later, against real hardware.

void lqr::reset() {}

lqr::Command lqr::update(const Setpoint&, const attitude::Estimate&,
                         const safety::Saturation&, float) {
    return {};
}
