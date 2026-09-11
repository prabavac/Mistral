// bmp388.h — BMP388 barometer driver — Mistral
//
// DISPLAY ONLY. The barometer is NOT altitude feedback: no control or fusion
// module may consume these values. They exist for the OLED readout.
//
// I2C on the shared sensor bus at cfg::BMP388_I2C_ADDR (0x76; 0x77 if SDO is
// pulled high). Uses the Adafruit BMP3XX library.
#pragma once

namespace bmp388 {

struct Reading {
    float pressure_pa;
    float temperature_c;
    float altitude_m;  // pressure altitude — display only
    bool  valid;
};

// Probe and configure the sensor. Returns false if it does not respond.
bool init();

// Latest sample.
Reading read();

}  // namespace bmp388
