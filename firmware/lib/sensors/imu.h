// imu.h: ICM-42688-P 6-axis IMU for Mistral (lib/sensors)
//
// I2C at cfg::ICM42688_I2C_ADDR (0x69) on Wire1 (cfg::I2C_SDA / I2C_SCL). Wire belongs to
// the OLED. Driver: finani/ICM42688 1.1.0, the version the bench sketch ran. Its begin()
// uses the bus as already started, so init() starts Wire1 first.
//
// Filtering happens ON THE CHIP: init() sets the UI filter from cfg::IMU_*_UI_FILT_* (sensors
// off while it changes, as the datasheet requires). No software filter on the gyro: phase lag
// inside the control loop costs stability margin.
//
// Named imu, not icm42688: on a case-insensitive filesystem icm42688.h and the library's
// ICM42688.h are the same file.
//
// Units: acceleration in g, angular rate in deg/s. SENSOR axes; remapping to the body
// frame is fusion's job.
#pragma once

namespace imu {

struct Vec3 {
    float x, y, z;
};

struct Reading {
    Vec3 accel_g;
    Vec3 gyro_dps;  // boot bias removed
    bool valid;     // false before a successful init() and on an I2C read error
};

// Start Wire1, probe and configure the sensor, wait cfg::IMU_SETTLE_MS, then average
// cfg::IMU_BIAS_SAMPLES readings for the gyro bias and the gravity direction. Blocks for
// several seconds (the library's begin() also calibrates): keep the vehicle upright and
// still. Returns false if the sensor does not respond; see beginCode().
bool init();

// Re-measure the gyro bias and gravity direction: the same cfg::IMU_BIAS_SAMPLES average as
// init(), blocking ~1.5 s. Keeps the previous values and returns false if the vehicle moved (any
// gyro axis spanning more than cfg::IMU_STILL_MAX_SPREAD_DPS) or the sensor never started.
bool calibrate();

// The library's begin() result: 1 ok, -3 WHO_AM_I mismatch (wrong bus, wrong address, or
// CS floating low, which puts the chip in SPI mode).
int beginCode();

// True if init() set the on-chip UI filter and read it back. False leaves the chip's reset
// filter (≈250 Hz) in place.
bool uiFilterSet();

Vec3 gyroBias_dps();
Vec3 accelMean_g();  // average during calibration: the gravity reference for fusion

// One burst read. Call once per control tick.
Reading read();

}  // namespace imu
