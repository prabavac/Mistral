// translation.h: lateral velocity/position estimator for Mistral (lib/fusion)
//
// MTF-01P flow + range → lateral (X/Y) velocity and position.
//
// Flow is ANGULAR. The sensor reports how fast the ground pattern sweeps across
// its view, so:  actual_velocity = reported_flow · altitude.
//
// Range is a SCALE FACTOR ONLY. Because it multiplies the whole estimate, noise
// on it goes straight into velocity, so low-pass it (cfg::RANGE_LPF_ALPHA) before
// use. Range is NEVER vertical/altitude feedback.
#pragma once

#include <mtf01.h>

namespace translation {

struct Estimate {
    float velX_mps, velY_mps;
    float posX_m, posY_m;
    bool  valid;  // false whenever the MTF-01P reading is invalid
};

// Zero position, clear the range low-pass state.
void reset();

// One estimator step. Call once per control tick; dt_s in seconds.
Estimate update(const mtf01::Reading& flow, float dt_s);

}  // namespace translation
