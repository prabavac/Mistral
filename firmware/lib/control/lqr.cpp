#include "lqr.h"

#include <cmath>
#include <mistral_config.h>

namespace {

// Live gains treat the solved K as decoupled: each input sees only its own axis's states.
static_assert(cfg::LQR_K[0][1] == 0.0f && cfg::LQR_K[0][3] == 0.0f && cfg::LQR_K[0][5] == 0.0f &&
                  cfg::LQR_K[1][0] == 0.0f && cfg::LQR_K[1][2] == 0.0f && cfg::LQR_K[1][4] == 0.0f,
              "lqr live gains need a decoupled LQR_K");

float ir[cfg::LQR_INPUTS] = {0.0f, 0.0f};  // rad·s

float clampAbs(float v, float limit) { return v < -limit ? -limit : (v > limit ? limit : v); }

float clampOr(float v, float lo, float hi, float current) {
    if (std::isnan(v)) return current;
    return v < lo ? lo : (v > hi ? hi : v);
}

lqr::AxisGains clampAxis(const lqr::AxisGains& g, const lqr::AxisGains& design,
                         const lqr::AxisGains& current) {
    return {clampOr(g.kth, design.kth * cfg::LQR_KTH_RANGE[0], design.kth * cfg::LQR_KTH_RANGE[1], current.kth),
            clampOr(g.kq, design.kq * cfg::LQR_KQ_RANGE[0], design.kq * cfg::LQR_KQ_RANGE[1], current.kq),
            clampOr(g.ki, design.ki * cfg::LQR_KI_RANGE[0], design.ki * cfg::LQR_KI_RANGE[1], current.ki)};
}

lqr::Gains live = lqr::bootGains();

}  // namespace

lqr::AxisGains lqr::designGains(uint8_t axis) {
    return {cfg::LQR_K[axis][axis], cfg::LQR_K[axis][2 + axis], cfg::LQR_K[axis][4 + axis]};
}

lqr::Gains lqr::bootGains() {
    AxisGains x = designGains(0), y = designGains(1);
    x.ki *= cfg::LQR_KI_BOOT_FACTOR;
    y.ki *= cfg::LQR_KI_BOOT_FACTOR;
    return {cfg::LQR_GAIN_SCALE, x, y};
}

lqr::Gains lqr::gains() { return live; }

lqr::Gains lqr::setGains(const Gains& g) {
    live = {clampOr(g.scale, 0.0f, cfg::LQR_GAIN_SCALE_MAX, live.scale),
            clampAxis(g.x, designGains(0), live.x),
            clampAxis(g.y, designGains(1), live.y)};
    return live;
}

void lqr::reset() { ir[0] = ir[1] = 0.0f; }

lqr::Output lqr::update(const attitude::Estimate& est, bool integrate,
                        const safety::Saturation& sat, float thrust_N, float dt_s) {
    const float thrustScale =
        cfg::THRUST_SCHED ? cfg::NOMINAL_THRUST_N / std::fmax(thrust_N, cfg::THRUST_LCLAMP_N) : 1.0f;
    const AxisGains g[cfg::LQR_INPUTS]        = {live.x, live.y};
    const float     r[cfg::LQR_INPUTS]        = {est.r1, est.r2};
    const float     dr[cfg::LQR_INPUTS]       = {est.dr1, est.dr2};
    const bool      servoSat[cfg::LQR_INPUTS] = {sat.tvcX, sat.tvcY};

    float uRaw[cfg::LQR_INPUTS], u[cfg::LQR_INPUTS];
    for (uint8_t i = 0; i < cfg::LQR_INPUTS; i++) {
        const bool integrating = integrate && g[i].ki > 0.0f;
        if (!integrating) ir[i] = 0.0f;

        uRaw[i] = -(g[i].kth * r[i] + g[i].kq * dr[i] + g[i].ki * ir[i]) * live.scale * thrustScale;
        u[i]    = clampAbs(uRaw[i], cfg::TVC_CLAMP_RAD);

        if (integrating) {
            const float inc = r[i] * dt_s;
            // ir enters u with a negative gain, so an increment drives u further into
            // saturation exactly when it has the opposite sign to u.
            const bool saturated = std::fabs(uRaw[i]) >= cfg::TVC_CLAMP_RAD || servoSat[i];
            if (!(saturated && inc * uRaw[i] < 0.0f)) ir[i] = clampAbs(ir[i] + inc, cfg::LQR_I_LIMIT);
        }
    }

    return {u[0], u[1], uRaw[0], uRaw[1], ir[0], ir[1]};
}
