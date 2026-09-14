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
    bool   valid;     // false until init() ran and Fusion's startup has converged and been zeroed
};

// Seed both estimators from the IMU's boot calibration (accelMean_g is the gravity
// reference). Call once in setup(), after imu::init() succeeded.
void init(const imu::Vec3& accelMean_g);

// One estimator step. Call once per control tick; dt_s in seconds. An invalid reading
// holds the previous estimate.
Estimate update(const imu::Reading& reading, float dt_s);

}  // namespace attitude
