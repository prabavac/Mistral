#include "throttle.h"

#include <Arduino.h>
#include <cmath>
#include <mistral_config.h>

namespace {

throttle::State state        = throttle::State::DISARMED;
uint32_t        armStartMs   = 0;
float           normalised   = 0.0f;
float           differential = 0.0f;
uint32_t        lastChangeMs = 0;  // millis() of the last change to `normalised`
bool            saturated    = false;

uint16_t esc1Us = 0, esc2Us = 0;
uint32_t esc1Duty = UINT32_MAX, esc2Duty = UINT32_MAX;  // last duty count written

uint32_t usToDuty(uint16_t us) {
    return (static_cast<uint32_t>(us) << cfg::PWM_RES_BITS) / cfg::PWM_PERIOD_US;
}

// One ESC's write, skipped if its duty count is unchanged.
void writeEsc(uint8_t pin, uint16_t us, uint16_t& lastUs, uint32_t& lastDuty) {
    const uint32_t duty = usToDuty(us);
    if (duty != lastDuty) {
        ledcWrite(pin, duty);
        lastDuty = duty;
    }
    lastUs = us;
}

float clampOrZero(float v, float lo, float hi) {
    if (std::isnan(v)) return 0.0f;
    return v < lo ? lo : (v > hi ? hi : v);
}

uint16_t fractionToUs(float f) {
    return cfg::ESC_MIN_US +
           static_cast<uint16_t>(lroundf(f * (cfg::ESC_MAX_US - cfg::ESC_MIN_US)));
}

void applyOutputs() {
    if (state != throttle::State::ARMED || normalised <= 0.0f) {
        saturated = false;
        writeEsc(cfg::ESC1_PIN, cfg::ESC_MIN_US, esc1Us, esc1Duty);
        writeEsc(cfg::ESC2_PIN, cfg::ESC_MIN_US, esc2Us, esc2Duty);
        return;
    }
    const float m1 = normalised + differential;
    const float m2 = normalised - differential;
    saturated = m1 < 0.0f || m1 > 1.0f || m2 < 0.0f || m2 > 1.0f;
    writeEsc(cfg::ESC1_PIN, fractionToUs(clampOrZero(m1, 0.0f, 1.0f)), esc1Us, esc1Duty);
    writeEsc(cfg::ESC2_PIN, fractionToUs(clampOrZero(m2, 0.0f, 1.0f)), esc2Us, esc2Duty);
}

// Calibration step: hold both ESCs at `us` for `ms`, printing a once-a-second countdown.
void holdBoth(uint16_t us, uint32_t ms, const char* prompt) {
    writeEsc(cfg::ESC1_PIN, us, esc1Us, esc1Duty);
    writeEsc(cfg::ESC2_PIN, us, esc2Us, esc2Duty);
    for (uint32_t left = ms; left > 0;) {
        Serial.printf("  %s  %lu s\n", prompt, static_cast<unsigned long>((left + 999) / 1000));
        const uint32_t step = left < 1000 ? left : 1000;
        delay(step);
        left -= step;
    }
}

}  // namespace

bool throttle::init() {
    if (!ledcAttachChannel(cfg::ESC1_PIN, cfg::PWM_FREQ_HZ, cfg::PWM_RES_BITS, cfg::ESC1_LEDC_CH)) return false;
    if (!ledcAttachChannel(cfg::ESC2_PIN, cfg::PWM_FREQ_HZ, cfg::PWM_RES_BITS, cfg::ESC2_LEDC_CH)) return false;
    disarm();
    return true;
}

bool throttle::arm() {
    if (state != State::DISARMED) return true;
    if (normalised > 0.0f) return false;
    state      = State::ARMING;
    armStartMs = millis();
    return true;
}

void throttle::disarm() {
    state        = State::DISARMED;
    differential = 0.0f;
    applyOutputs();
}

void throttle::setNormalised(float value) {
    value = clampOrZero(value, 0.0f, 1.0f);
    if (value != normalised) {
        normalised   = value;
        lastChangeMs = millis();
    }
    applyOutputs();
}

void throttle::setDifferential(float delta) {
    differential = clampOrZero(delta, -1.0f, 1.0f);
    applyOutputs();
}

void throttle::update() {
    const uint32_t now = millis();
    if (state == State::ARMING) {
        if (normalised > 0.0f) {
            disarm();  // throttle moved off zero during the hold
        } else if (now - armStartMs >= cfg::ARM_HOLD_MS) {
            state = State::ARMED;
        }
    }
    if (state == State::ARMED && normalised > 0.0f &&
        now - lastChangeMs >= cfg::THROTTLE_AUTOCUT_MS) {
        disarm();
    }
    applyOutputs();
}

bool throttle::calibrate() {
    if (state != State::DISARMED) return false;
    Serial.println(F("ESC calibration — PROPS OFF"));
    holdBoth(cfg::ESC_MIN_US, cfg::ESC_CAL_UNPLUG_MS, "MIN  unplug the battery");
    holdBoth(cfg::ESC_MAX_US, cfg::ESC_CAL_MAX_MS,    "MAX  PLUG IN NOW");
    holdBoth(cfg::ESC_MIN_US, cfg::ESC_CAL_MIN_MS,    "MIN  low point");
    Serial.println(F("ESC calibration done"));
    return true;
}

throttle::Status throttle::status() {
    uint32_t armRemainingMs = 0;
    if (state == State::ARMING) {
        const uint32_t elapsed = millis() - armStartMs;
        armRemainingMs         = elapsed < cfg::ARM_HOLD_MS ? cfg::ARM_HOLD_MS - elapsed : 0;
    }
    return {state, armRemainingMs, normalised, differential, esc1Us, esc2Us, saturated};
}
