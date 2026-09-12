#include "tvc.h"

#include <Arduino.h>
#include <cmath>
#include <mistral_config.h>

namespace {

struct Servo {
    uint8_t              pin;
    uint8_t              channel;
    const cfg::ServoCal& cal;
    int16_t              trimUs;
    float                deg;        // last commanded TVC deflection, after the clamp
    uint16_t             us;         // last pulse commanded
    uint32_t             duty;       // last duty count written; UINT32_MAX before the first write
    bool                 saturated;
};

Servo servoX{cfg::SERVO_X_PIN, cfg::SERVO_X_LEDC_CH, cfg::SERVO_X_CAL, 0, 0.0f, 0, UINT32_MAX, false};
Servo servoY{cfg::SERVO_Y_PIN, cfg::SERVO_Y_LEDC_CH, cfg::SERVO_Y_CAL, 0, 0.0f, 0, UINT32_MAX, false};

uint32_t usToDuty(uint16_t us) {
    return (static_cast<uint32_t>(us) << cfg::PWM_RES_BITS) / cfg::PWM_PERIOD_US;
}

// TVC degrees → pulse for one servo, then write it if the duty count changed.
void command(Servo& s, float tvcDeg) {
    bool sat = false;
    if (std::isnan(tvcDeg)) tvcDeg = 0.0f;
    if (tvcDeg > cfg::TVC_CLAMP_DEG)  { tvcDeg = cfg::TVC_CLAMP_DEG;  sat = true; }
    if (tvcDeg < -cfg::TVC_CLAMP_DEG) { tvcDeg = -cfg::TVC_CLAMP_DEG; sat = true; }

    float us = s.cal.centreUs + s.trimUs +
               tvcDeg * s.cal.gearRatio * s.cal.dir * s.cal.usPerServoDeg;
    if (us < s.cal.minUs) { us = s.cal.minUs; sat = true; }
    if (us > s.cal.maxUs) { us = s.cal.maxUs; sat = true; }

    s.deg       = tvcDeg;
    s.us        = static_cast<uint16_t>(lroundf(us));
    s.saturated = sat;

    const uint32_t duty = usToDuty(s.us);
    if (duty != s.duty) {
        ledcWrite(s.pin, duty);
        s.duty = duty;
    }
}

}  // namespace

bool tvc::init() {
    if (!ledcAttachChannel(servoX.pin, cfg::PWM_FREQ_HZ, cfg::PWM_RES_BITS, servoX.channel)) return false;
    if (!ledcAttachChannel(servoY.pin, cfg::PWM_FREQ_HZ, cfg::PWM_RES_BITS, servoY.channel)) return false;
    centre();
    return true;
}

void tvc::centre() { setDeflection(0.0f, 0.0f); }

void tvc::setDeflection(float pitchDeg, float rollDeg) {
    command(servoX, pitchDeg);
    command(servoY, rollDeg);
}

void tvc::setTrim(Axis axis, int16_t us) {
    Servo& s = (axis == Axis::X) ? servoX : servoY;
    if (us == s.trimUs) return;
    s.trimUs = us;
    command(s, s.deg);
}

tvc::Status tvc::status() {
    return {servoX.deg,    servoY.deg,    servoX.us,        servoY.us,
            servoX.trimUs, servoY.trimUs, servoX.saturated, servoY.saturated};
}
