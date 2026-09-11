#include "safety.h"

safety::Saturation safety::saturation(const tvc::Status& t, const motors::Status& m) {
    return {t.x_saturated, t.y_saturated, m.saturated};
}

// STUB — refuses to arm until the gates are implemented.
bool safety::armingAllowed(const ArmInputs&) { return false; }
