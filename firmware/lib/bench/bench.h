// bench.h — bring-up bench tests — Mistral
//
// Temporary sequences that exercise a subsystem on the stand before the flight code that
// will drive it exists. Each is switched from mistral_config.h; delete the call in
// main.cpp once a controller drives that actuator.
#pragma once

#include <cstdint>

namespace bench {

// TVC diagonal sweep: both axes to +cfg::TVC_CLAMP_DEG, hold cfg::BENCH_SWEEP_DWELL_MS,
// then both to −cfg::TVC_CLAMP_DEG, hold, repeat. Equal pitch and roll deflection, so the
// nozzle travels the diagonal. Commands go through tvc::setDeflection — the real clamp,
// gear ratio, sign and µs backstop path — and each end is commanded directly, with no
// interpolation: the servos slew there natively. Call every control tick; tvc only
// writes when the pulse changes.
void servoSweep(uint32_t nowMs);

}  // namespace bench
