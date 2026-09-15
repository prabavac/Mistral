#include "attitude.h"

#include <Fusion.h>
#include <cmath>
#include <mistral_config.h>

namespace {

constexpr float DEG2RAD = 0.017453293f;

using V3 = imu::Vec3;

float dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

V3 cross(const V3& a, const V3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

V3 norm(const V3& v) {
    const float n = std::sqrt(dot(v, v));
    return n > 1e-6f ? V3{v.x / n, v.y / n, v.z / n} : V3{0.0f, 0.0f, 1.0f};
}

float clampUnit(float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); }

// Complementary filter. Sensing axes come from the gravity vector captured at boot, so any
// mounting angle works: e1 = sensor X orthogonalised against gravity (the mount's rotation
// axis, body X), e2 = gRef × e1 (body Y). Fixed board axes BREAK with the IMU on its side —
// with gravity on sensor Y, one tilt direction reads nothing at all.
V3    gRef{0.0f, 0.0f, 1.0f}, e1{1.0f, 0.0f, 0.0f}, e2{0.0f, 1.0f, 0.0f};
float comp1 = 0.0f, comp2 = 0.0f;

FusionAhrs ahrs;
FusionBias offsetTracker;  // gyro offset left after calibration, deg/s, sensor axes
bool       fusionZeroed = false;
float      zeroRoll_deg = 0.0f, zeroPitch_deg = 0.0f;

attitude::Estimate last{};

}  // namespace

void attitude::init(const imu::Vec3& accelMean_g) {
    gRef          = norm(accelMean_g);
    const V3    x = {1.0f, 0.0f, 0.0f};
    const float p = dot(x, gRef);
    e1            = norm({x.x - gRef.x * p, x.y - gRef.y * p, x.z - gRef.z * p});
    e2            = norm(cross(gRef, e1));
    comp1 = comp2 = 0.0f;

    FusionAhrsInitialise(&ahrs);
    const FusionAhrsSettings settings = {
        .sampleRate            = static_cast<float>(cfg::CONTROL_LOOP_HZ),
        .convention            = FusionConventionNwu,
        .gain                  = cfg::FUSION_GAIN,
        .gyroscopeRange        = static_cast<float>(cfg::IMU_GYRO_RANGE_DPS),
        .accelerationRejection = cfg::FUSION_ACCEL_REJECTION_DEG,
        .magneticRejection     = 0.0f,  // no magnetometer
        .rejectionTimeout      = cfg::FUSION_REJECTION_TIMEOUT_S,
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    FusionBiasInitialise(&offsetTracker);
    const FusionBiasSettings biasSettings = {
        .sampleRate          = static_cast<float>(cfg::CONTROL_LOOP_HZ),
        .stationaryThreshold = cfg::FUSION_BIAS_STILL_DPS,
        .stationaryPeriod    = cfg::FUSION_BIAS_STILL_S,
    };
    FusionBiasSetSettings(&offsetTracker, &biasSettings);
    fusionZeroed = false;
}

attitude::Estimate attitude::update(const imu::Reading& reading, float dt_s, bool learnGyroOffset) {
    if (!reading.valid) return last;
    // Remove the gyro offset that drifted since calibration. FusionBiasUpdate learns (only once
    // the vehicle has been still for cfg::FUSION_BIAS_STILL_S) and removes; in flight, just remove.
    const FusionVector raw  = {{reading.gyro_dps.x, reading.gyro_dps.y, reading.gyro_dps.z}};
    const FusionVector gyro = learnGyroOffset
                                  ? FusionBiasUpdate(&offsetTracker, raw)
                                  : FusionVectorSubtract(raw, FusionBiasGetOffset(&offsetTracker));
    const V3 g = {gyro.axis.x, gyro.axis.y, gyro.axis.z};

    // Complementary filter. The rotation from captured-up to current-up has magnitude
    // sin(angle); projecting it onto e1/e2 gives the tilt about each sensing axis.
    const V3    c     = cross(gRef, norm(reading.accel_g));
    const float acc1  = -std::asin(clampUnit(dot(c, e1)));
    const float acc2  = -std::asin(clampUnit(dot(c, e2)));
    const float rate1 = dot(g, e1) * DEG2RAD;
    const float rate2 = dot(g, e2) * DEG2RAD;
    comp1 = cfg::COMP_FILTER_A * (comp1 + rate1 * dt_s) + (1.0f - cfg::COMP_FILTER_A) * acc1;
    comp2 = cfg::COMP_FILTER_A * (comp2 + rate2 * dt_s) + (1.0f - cfg::COMP_FILTER_A) * acc2;

    // Fusion. The IMU is mounted rotated 90° about its X axis — body X = sensor +X, body Y =
    // sensor +Z, body Z = sensor −Y — so gyro AND accel get the same remap.
    const FusionVector bodyGyro =
        FusionRemap(FusionVector{{g.x, g.y, g.z}}, FusionRemapAlignmentPXPZNY);
    const FusionVector bodyAccel = FusionRemap(
        FusionVector{{reading.accel_g.x, reading.accel_g.y, reading.accel_g.z}},
        FusionRemapAlignmentPXPZNY);
    FusionAhrsSetSamplePeriod(&ahrs, dt_s);
    FusionAhrsUpdateNoMagnetometer(&ahrs, bodyGyro, bodyAccel);
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    // Zero at boot, like the complementary filter, once Fusion's startup ramp has converged:
    // both estimators share one reference, and IMU mounting error isn't read as tilt.
    if (!fusionZeroed && !FusionAhrsGetFlags(&ahrs).startup) {
        zeroRoll_deg  = euler.angle.roll;
        zeroPitch_deg = euler.angle.pitch;
        fusionZeroed  = true;
    }

    last.comp   = {comp1, comp2};
    last.fusion = {(euler.angle.roll - zeroRoll_deg) * DEG2RAD,
                   (euler.angle.pitch - zeroPitch_deg) * DEG2RAD};
    if (cfg::ATTITUDE_USE_FUSION) {
        last.r1  = last.fusion.r1;
        last.r2  = last.fusion.r2;
        last.dr1 = bodyGyro.axis.x * DEG2RAD;
        last.dr2 = bodyGyro.axis.y * DEG2RAD;
    } else {
        last.r1  = comp1;
        last.r2  = comp2;
        last.dr1 = rate1;
        last.dr2 = rate2;
    }
    const FusionVector offset =
        FusionRemap(FusionBiasGetOffset(&offsetTracker), FusionRemapAlignmentPXPZNY);
    last.gyroOffset1Dps = offset.axis.x;
    last.gyroOffset2Dps = offset.axis.y;
    last.yawRate = bodyGyro.axis.z * DEG2RAD;
    last.valid   = fusionZeroed;
    return last;
}
