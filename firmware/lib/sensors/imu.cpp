#include "imu.h"

#include <Arduino.h>
#include <ICM42688.h>
#include <Wire.h>
#include <mistral_config.h>

namespace {

// Two-argument constructor: library 1.1.0's begin() calls Wire1.begin() with no pins, so
// it keeps the bus init() started on 41/42.
ICM42688 sensor(Wire1, cfg::ICM42688_I2C_ADDR);

int       beginResult = 0;
bool      uiFilter    = false;
imu::Vec3 gyroBias{}, accelMean{};

// Bank-0 registers (datasheet DS-000347 §14). Library 1.1.0 has no setter for the UI filter.
constexpr uint8_t REG_PWR_MGMT0         = 0x4E;  // bits 3:2 gyro mode, 1:0 accel mode
constexpr uint8_t REG_GYRO_CONFIG1      = 0x51;  // bits 3:2 GYRO_UI_FILT_ORD
constexpr uint8_t REG_GYRO_ACCEL_CONFIG0 = 0x52;  // bits 7:4 ACCEL_UI_FILT_BW, 3:0 GYRO_UI_FILT_BW
constexpr uint8_t REG_ACCEL_CONFIG1     = 0x53;  // bits 4:3 ACCEL_UI_FILT_ORD

bool readReg(uint8_t reg, uint8_t& value) {
    Wire1.beginTransmission(cfg::ICM42688_I2C_ADDR);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom(cfg::ICM42688_I2C_ADDR, static_cast<uint8_t>(1)) != 1) return false;
    value = Wire1.read();
    return true;
}

bool writeReg(uint8_t reg, uint8_t value) {
    Wire1.beginTransmission(cfg::ICM42688_I2C_ADDR);
    Wire1.write(reg);
    Wire1.write(value);
    return Wire1.endTransmission() == 0;
}

// Replace only the `mask` bits, keeping the rest (reserved bits included).
bool updateReg(uint8_t reg, uint8_t mask, uint8_t bits) {
    uint8_t value;
    return readReg(reg, value) && writeReg(reg, (value & ~mask) | (bits & mask));
}

// The datasheet (§12.9) forbids changing filter registers while the sensors run: turn them
// off, change only the filter bits, turn them back on, then read back.
bool configureUiFilter() {
    static_assert(cfg::IMU_GYRO_UI_FILT_BW <= 7 && cfg::IMU_ACCEL_UI_FILT_BW <= 7 &&
                      cfg::IMU_UI_FILT_ORDER <= 2,
                  "UI filter BW index is 0..7, order 0..2");
    const uint8_t bw = static_cast<uint8_t>((cfg::IMU_ACCEL_UI_FILT_BW << 4) | cfg::IMU_GYRO_UI_FILT_BW);

    uint8_t pwr;
    if (!readReg(REG_PWR_MGMT0, pwr) || !writeReg(REG_PWR_MGMT0, pwr & ~0x0F)) return false;
    delay(1);
    const bool ok = updateReg(REG_GYRO_CONFIG1, 0x0C, cfg::IMU_UI_FILT_ORDER << 2) &&
                    updateReg(REG_ACCEL_CONFIG1, 0x18, cfg::IMU_UI_FILT_ORDER << 3) &&
                    writeReg(REG_GYRO_ACCEL_CONFIG0, bw);
    writeReg(REG_PWR_MGMT0, pwr);  // back on in the mode begin() chose, even if a write failed
    delay(50);                     // gyro start-up is 30 ms

    uint8_t readBack;
    return ok && readReg(REG_GYRO_ACCEL_CONFIG0, readBack) && readBack == bw;
}

}  // namespace

bool imu::init() {
    Wire1.begin(cfg::I2C_SDA, cfg::I2C_SCL);  // must precede sensor.begin()
    Wire1.setClock(cfg::I2C_CLOCK_HZ);
    beginResult = sensor.begin();
    if (beginResult < 0) return false;

    static_assert(cfg::IMU_ACCEL_RANGE_G == 8 && cfg::IMU_GYRO_RANGE_DPS == 500,
                  "change the setAccelFS/setGyroFS calls to match");
    sensor.setAccelFS(ICM42688::gpm8);
    sensor.setGyroFS(ICM42688::dps500);
    sensor.setAccelODR(ICM42688::odr1k);
    sensor.setGyroODR(ICM42688::odr1k);
    uiFilter = configureUiFilter();
    delay(cfg::IMU_SETTLE_MS);

    // Gyro bias MUST be removed: left in, it integrates into an angle that creeps and never
    // returns to zero. begin() already subtracts the library's estimate; this average (the
    // bench sketch's) removes what is left.
    Vec3 a{}, g{};
    for (uint16_t i = 0; i < cfg::IMU_BIAS_SAMPLES; i++) {
        sensor.getAGT();
        a.x += sensor.accX();
        a.y += sensor.accY();
        a.z += sensor.accZ();
        g.x += sensor.gyrX();
        g.y += sensor.gyrY();
        g.z += sensor.gyrZ();
        delay(cfg::IMU_BIAS_SAMPLE_MS);
    }
    const float n = cfg::IMU_BIAS_SAMPLES;
    gyroBias      = {g.x / n, g.y / n, g.z / n};
    accelMean     = {a.x / n, a.y / n, a.z / n};
    return true;
}

int imu::beginCode() { return beginResult; }

bool imu::uiFilterSet() { return uiFilter; }

imu::Vec3 imu::gyroBias_dps() { return gyroBias; }

imu::Vec3 imu::accelMean_g() { return accelMean; }

imu::Reading imu::read() {
    if (beginResult != 1) return {};
    const bool ok = sensor.getAGT() > 0;
    return {{sensor.accX(), sensor.accY(), sensor.accZ()},
            {sensor.gyrX() - gyroBias.x, sensor.gyrY() - gyroBias.y, sensor.gyrZ() - gyroBias.z},
            ok};
}
