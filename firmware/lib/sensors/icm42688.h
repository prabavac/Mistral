// icm42688.h — ICM-42688-P 6-axis IMU driver — Mistral
//
// Bring-up runs on I2C at cfg::ICM42688_I2C_ADDR (0x68) on the shared sensor bus
// (cfg::I2C_SDA / cfg::I2C_SCL). SPI is the preferred flight interface — faster,
// and off the shared I2C bus — and replaces I2C once SPI pins are assigned in
// mistral_config.h. The driver library is chosen during bring-up (not yet in
// lib_deps).
//
// Units: acceleration m/s², angular rate deg/s. There is no magnetometer on this
// build, so nothing downstream can know absolute heading.
#pragma once

namespace icm42688 {

struct Reading {
    float ax_mps2, ay_mps2, az_mps2;
    float gx_dps, gy_dps, gz_dps;
    bool  valid;  // false until a good sample, and on any bus error
};

// Probe and configure the sensor. Hold the vehicle still (gyro bias capture).
// Returns false if the sensor does not respond. Call once in setup().
bool init();

// Latest sample.
Reading read();

}  // namespace icm42688
