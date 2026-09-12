#include "bench.h"

#include <mistral_config.h>
#include <tvc.h>

void bench::servoSweep(uint32_t nowMs) {
    const bool  positive = (nowMs / cfg::BENCH_SWEEP_DWELL_MS) % 2 == 0;
    const float deg      = positive ? cfg::TVC_CLAMP_DEG : -cfg::TVC_CLAMP_DEG;
    tvc::setDeflection(deg, deg);
}
