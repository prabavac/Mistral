// attitude.h — attitude estimator — Mistral (lib/fusion)
//
// IMU → tilt angles r1, r2 and body rates dr1, dr2 in RADIANS, the controller's units.
//   r1 = rotation about body X (drives servo X), r2 = rotation about body Y (servo Y),
//   both relative to the pose at boot. The repo calls servo X "pitch" while Fusion calls
//   rotation about X "roll" — r1/r2 sidestep both names.
//
// Two estimators run every tick so they can be compared on the same tilts:
//   - complementary filter — the bench sketch's, gravity-referenced, verified on hardware
//   - Fusion AHRS (xioTechnologies, vendored in lib/FusionAhrs), zeroed at boot
// cfg::ATTITUDE_USE_FUSION picks which one feeds r1/r2/dr1/dr2.
//
// Rates are the bias-corrected gyro, unfiltered: software filtering in the control path
// costs phase margin.
//
// Gyro offset: imu removes the bias measured at calibration. What drifts after that (the IMU
// warming up) is tracked here by Fusion's FusionBias while the vehicle sits still, and removed
// before both estimators — Fusion at gain 0.5 would otherwise hold ~2° of tilt per 1 °/s.
//
// Yaw is RATE ONLY. There is no magnetometer on this build, so absolute heading is
// unobservable — do not estimate or control a yaw angle.
#pragma once

#include <imu.h>

namespace attitude {

struct Angles {
    float r1, r2;  // rad
};

struct Estimate {
    float  r1, r2;    // rad — from the selected estimator
    float  dr1, dr2;  // rad/s — body rates about X and Y
    float  yawRate;   // rad/s — about body Z
    Angles comp;      // complementary filter, always computed
    Angles fusion;    // Fusion AHRS, always computed
    float  gyroOffset1Dps, gyroOffset2Dps;  // run-time tracked gyro offset about body X / Y, deg/s
    bool   valid;     // false until init() ran and Fusion's startup has converged and been zeroed
};

// Seed both estimators from the IMU's calibration (accelMean_g is the gravity reference) and
// reset the gyro offset tracker. Call in setup() after imu::init() succeeded, and again after
// imu::calibrate() to re-zero on the ground. valid stays false until Fusion has re-converged.
void init(const imu::Vec3& accelMean_g);

// One estimator step. Call once per control tick; dt_s in seconds. An invalid reading
// holds the previous estimate. learnGyroOffset lets the offset tracker learn from still periods
// (pass false while FLYING); the offset learned so far is removed either way.
Estimate update(const imu::Reading& reading, float dt_s, bool learnGyroOffset);

}  // namespace attitude
