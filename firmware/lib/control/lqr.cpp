#include "lqr.h"

#include <cmath>
#include <mistral_config.h>

namespace {

float ir[cfg::LQR_INPUTS] = {0.0f, 0.0f};  // rad·s

float clampAbs(float v, float limit) { return v < -limit ? -limit : (v > limit ? limit : v); }

}  // namespace

void lqr::reset() { ir[0] = ir[1] = 0.0f; }

lqr::Output lqr::update(const attitude::Estimate& est, bool integrate,
                        const safety::Saturation& sat, float thrust_N, float dt_s) {
    if (!integrate) reset();

    const float x[cfg::LQR_STATES] = {est.r1, est.r2, est.dr1, est.dr2, ir[0], ir[1]};
    const float thrustScale =
        cfg::THRUST_SCHED ? cfg::NOMINAL_THRUST_N / std::fmax(thrust_N, cfg::THRUST_LCLAMP_N) : 1.0f;

    float uRaw[cfg::LQR_INPUTS], u[cfg::LQR_INPUTS];
    for (uint8_t i = 0; i < cfg::LQR_INPUTS; i++) {
        float s = 0.0f;
        for (uint8_t j = 0; j < cfg::LQR_STATES; j++) s -= cfg::LQR_K[i][j] * x[j];
        uRaw[i] = s * cfg::LQR_GAIN_SCALE * thrustScale;
        u[i]    = clampAbs(uRaw[i], cfg::TVC_CLAMP_RAD);
    }

    if (integrate) {
        const float r[cfg::LQR_INPUTS]        = {est.r1, est.r2};
        const bool  servoSat[cfg::LQR_INPUTS] = {sat.tvcX, sat.tvcY};
        for (uint8_t i = 0; i < cfg::LQR_INPUTS; i++) {
            const float inc = r[i] * dt_s;
            // ir enters u with a negative gain, so an increment drives u further into
            // saturation exactly when it has the opposite sign to u.
            const bool saturated = std::fabs(uRaw[i]) >= cfg::TVC_CLAMP_RAD || servoSat[i];
            if (!(saturated && inc * uRaw[i] < 0.0f)) {
                ir[i] = clampAbs(ir[i] + inc, cfg::LQR_I_LIMIT);
            }
        }
    }

    return {u[0], u[1], uRaw[0], uRaw[1], ir[0], ir[1]};
}
