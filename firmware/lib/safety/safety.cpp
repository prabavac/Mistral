#include "safety.h"

safety::Saturation safety::saturation(const tvc::Status& t, const throttle::Status& m) {
    return {t.x_saturated, t.y_saturated, m.saturated};
}

bool safety::armingAllowed(const ArmInputs& in) {
    return in.imuValid && in.attitudeValid && in.commonThrust <= 0.0f;
}
