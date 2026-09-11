// attitude.h — attitude estimator — Mistral (lib/fusion)
//
// IMU → pitch and roll (degrees) and yaw RATE (deg/s).
//
// Yaw is RATE ONLY. There is no magnetometer on this build, so absolute heading
// is unobservable — do not estimate or control a yaw angle.
#pragma once

#include <icm42688.h>

namespace attitude {

struct Estimate {
    float pitch_deg;
    float roll_deg;
    float yawRate_dps;
    bool  valid;  // false until the filter has been seeded from a valid IMU sample
};

// Forget filter state; the next update() re-seeds.
void reset();

// One estimator step. Call once per control tick; dt_s in seconds.
Estimate update(const icm42688::Reading& imu, float dt_s);

}  // namespace attitude
