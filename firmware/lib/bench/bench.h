// bench.h — bring-up bench tests — Mistral
//
// Temporary sequences that exercise a subsystem on the stand before the flight code that
// will drive it exists. Each is switched from mistral_config.h; delete the call in
// main.cpp once it is no longer needed.
#pragma once

#include <cstdint>

namespace bench {

// Gear-ratio measurement sweep, one axis at a time: X to +cfg::TVC_CLAMP_RAD, X to −, then
// Y to +, Y to −, each held cfg::BENCH_SWEEP_DWELL_MS, repeating; the idle axis sits at 0.
// Measure each nozzle's real deflection against the commanded ±8.59° (bench/VERIFY.md).
// Commands go through tvc::setDeflection — the real clamp, gear ratio, sign and µs
// backstop path — and each target is commanded directly, with no interpolation. Call
// every control tick while DISARMED.
void servoSweep(uint32_t nowMs);

}  // namespace bench
