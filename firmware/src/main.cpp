// main.cpp — Mistral thrust-vectored coaxial drone
//
// Wiring only. setup() brings modules up in a safe order; loop() runs at
// cfg::CONTROL_LOOP_HZ on core 1, applies ground-station commands to the ESCs, and — while
// cfg::BENCH_SERVO_SWEEP is set — sweeps the gimbal. No control law yet: estimators, LQR
// and the state machine are still stubs.
//
// Every module header is included so `pio run` compiles every library — the
// dependency finder only builds libraries reachable from src/.

#include <Arduino.h>
#include <Wire.h>

#include <mistral_config.h>

#include <attitude.h>
#include <bench.h>
#include <bmp388.h>
#include <display.h>
#include <icm42688.h>
#include <lqr.h>
#include <mtf01.h>
#include <safety.h>
#include <state_machine.h>
#include <throttle.h>
#include <translation.h>
#include <tvc.h>
#include <wifi_link.h>

// A failed LEDC attach produces no pulses at all while everything else looks alive.
// Stop here instead of running.
static void halt(const char* why) {
    for (;;) {
        Serial.println(why);
        delay(1000);
    }
}

void setup() {
    Serial.begin(cfg::SERIAL_BAUD);

    if (!throttle::init()) halt("HALT: throttle::init - LEDC attach failed");  // FIRST: both ESCs at min
    if (!tvc::init()) halt("HALT: tvc::init - LEDC attach failed");            // servos parked at centre
    state_machine::init();

    display::init();

    Wire.begin(cfg::I2C_SDA, cfg::I2C_SCL);
    icm42688::init();
    bmp388::init();
    mtf01::init();

    if (!wifi_link::init()) halt("HALT: wifi_link::init - SoftAP failed");
}

void loop() {
    static uint32_t lastTickMs = 0;
    const uint32_t  now        = millis();
    if (now - lastTickMs < 1000 / cfg::CONTROL_LOOP_HZ) return;
    lastTickMs = now;

    // Ground-station commands: recorded by the network side, applied here at the loop's
    // own rate. The throttle level goes first, so an arm is judged against the zero the
    // network side set when it received the arm. Kill is applied last so it always wins.
    const wifi_link::Commands cmd = wifi_link::takeCommands();
    throttle::setNormalised(cmd.throttle);
    if (cmd.arm) {
        const bool ok = throttle::arm();
        wifi_link::ack(cmd.armId, ok, "throttle not zero");
    }
    if (cmd.disarm) {
        throttle::disarm();
        wifi_link::ack(cmd.disarmId, true);
    }
    tvc::setTrim(tvc::Axis::X, cmd.trimXUs);
    tvc::setTrim(tvc::Axis::Y, cmd.trimYUs);
    if (!cmd.linkAlive && throttle::status().state != throttle::State::DISARMED) {
        Serial.println("Link lost - disarmed");
        throttle::disarm();
    }
    if (cmd.kill) {
        throttle::disarm();
        wifi_link::ack(cmd.killId, true);
    }

    throttle::update();

    // Bring-up: no controller yet, so the bench sweep drives the gimbal.
    if (cfg::BENCH_SERVO_SWEEP) bench::servoSweep(now);

    wifi_link::publish({now, throttle::status(), tvc::status()});
}
