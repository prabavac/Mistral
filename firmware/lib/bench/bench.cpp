#include "bench.h"

#include <mistral_config.h>
#include <tvc.h>

void bench::servoSweep(uint32_t nowMs) {
    constexpr float        L         = cfg::TVC_CLAMP_RAD;
    static constexpr float STEPS[4][2] = {{L, 0.0f}, {-L, 0.0f}, {0.0f, L}, {0.0f, -L}};
    const auto&            step      = STEPS[(nowMs / cfg::BENCH_SWEEP_DWELL_MS) % 4];
    tvc::setDeflection(step[0], step[1]);
}
