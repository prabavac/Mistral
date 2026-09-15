#include "tvc.h"

#include <Arduino.h>
#include <cmath>
#include <mistral_config.h>

namespace {

constexpr float RAD2DEG = 57.29577951f;

struct Servo {
    uint8_t              pin;
    uint8_t              channel;
    const cfg::ServoCal& cal;
    int16_t              trimUs;
    float                rad;        // last commanded nozzle deflection, after the clamp
    float                targetUs;   // pulse for that deflection
    float                writtenUs;  // last pulse actually written
    uint32_t             duty;       // last duty count written; UINT32_MAX before the first write
    bool                 saturated;
};

Servo servoX{cfg::SERVO_X_PIN, cfg::SERVO_X_LEDC_CH, cfg::SERVO_X_CAL, 0, 0.0f, 0.0f, -1.0f, UINT32_MAX, false};
Servo servoY{cfg::SERVO_Y_PIN, cfg::SERVO_Y_LEDC_CH, cfg::SERVO_Y_CAL, 0, 0.0f, 0.0f, -1.0f, UINT32_MAX, false};

uint32_t lastFrameUs = 0;
bool     anyFrame    = false;

uint32_t usToDuty(float us) {
    return (static_cast<uint32_t>(lroundf(us)) << cfg::PWM_RES_BITS) / cfg::PWM_PERIOD_US;
}

// Nozzle radians → target pulse for one servo.
void compute(Servo& s, float rad) {
    bool sat = false;
    if (std::isnan(rad)) rad = 0.0f;
    if (rad > cfg::TVC_CLAMP_RAD)  { rad = cfg::TVC_CLAMP_RAD;  sat = true; }
    if (rad < -cfg::TVC_CLAMP_RAD) { rad = -cfg::TVC_CLAMP_RAD; sat = true; }

    float us = s.cal.centreUs + s.trimUs +
               rad * RAD2DEG * s.cal.gearRatio * s.cal.dir * s.cal.usPerServoDeg;
    if (us < s.cal.minUs) { us = s.cal.minUs; sat = true; }
    if (us > s.cal.maxUs) { us = s.cal.maxUs; sat = true; }

    s.rad       = rad;
    s.targetUs  = us;
    s.saturated = sat;
}

// Write on change only, with the MG90S's dead band.
void write(Servo& s) {
    if (std::fabs(s.targetUs - s.writtenUs) < cfg::SERVO_DEADBAND_US) return;
    const uint32_t duty = usToDuty(s.targetUs);
    if (duty == s.duty) return;
    ledcWrite(s.pin, duty);
    s.duty      = duty;
    s.writtenUs = s.targetUs;
}

// At most one write per 50 Hz PWM frame, for both servos.
void writeFrame() {
    const uint32_t now = micros();
    if (anyFrame && now - lastFrameUs < cfg::PWM_PERIOD_US) return;
    lastFrameUs = now;
    anyFrame    = true;
    write(servoX);
    write(servoY);
}

}  // namespace

bool tvc::init() {
    if (!ledcAttachChannel(servoX.pin, cfg::PWM_FREQ_HZ, cfg::PWM_RES_BITS, servoX.channel)) return false;
    if (!ledcAttachChannel(servoY.pin, cfg::PWM_FREQ_HZ, cfg::PWM_RES_BITS, servoY.channel)) return false;
    centre();
    return true;
}

void tvc::centre() { setDeflection(0.0f, 0.0f); }

void tvc::setDeflection(float xRad, float yRad) {
    compute(servoX, xRad);
    compute(servoY, yRad);
    writeFrame();
}

void tvc::setTrim(Axis axis, int16_t us) {
    Servo& s = (axis == Axis::X) ? servoX : servoY;
    if (us > cfg::TVC_TRIM_MAX_US) us = cfg::TVC_TRIM_MAX_US;
    if (us < -cfg::TVC_TRIM_MAX_US) us = -cfg::TVC_TRIM_MAX_US;
    if (us == s.trimUs) return;
    s.trimUs = us;
    compute(s, s.rad);
    writeFrame();
}

tvc::Status tvc::status() {
    return {servoX.rad,
            servoY.rad,
            static_cast<uint16_t>(lroundf(servoX.targetUs)),
            static_cast<uint16_t>(lroundf(servoY.targetUs)),
            servoX.trimUs,
            servoY.trimUs,
            servoX.saturated,
            servoY.saturated};
}
