// main.cpp — Mistral thrust-vectored coaxial drone
//
// Wiring only. setup() brings modules up in a safe order; loop() will call them at
// cfg::CONTROL_LOOP_HZ. No logic lives here — it belongs in firmware/lib/.
//
// SCAFFOLD: every module is a stub. The control loop is written later, one module
// at a time, against real hardware.
//
// Every module header is included so `pio run` compiles every library — the
// dependency finder only builds libraries reachable from src/.

#include <Arduino.h>
#include <Wire.h>

#include <mistral_config.h>

#include <attitude.h>
#include <bmp388.h>
#include <display.h>
#include <icm42688.h>
#include <lqr.h>
#include <motors.h>
#include <mtf01.h>
#include <safety.h>
#include <state_machine.h>
#include <translation.h>
#include <tvc.h>

void setup() {
    Serial.begin(cfg::SERIAL_BAUD);

    motors::init();         // FIRST: both ESCs held at ESC_MIN_US
    tvc::init();            // servos attached once and centered
    state_machine::init();  // DISARMED

    display::init();

    Wire.begin(cfg::I2C_SDA, cfg::I2C_SCL);
    icm42688::init();
    bmp388::init();
    mtf01::init();
}

void loop() {
    // Control loop — not yet written.
}
