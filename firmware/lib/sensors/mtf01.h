// mtf01.h — MicoAir MTF-01P optical flow + rangefinder driver — Mistral
//
// One module, two measurements: angular optical flow and a downward range (up to
// 12 m), over UART at cfg::MTF01_BAUD (115200).
//
// NON-BLOCKING. update() parses only the bytes already sitting in the UART buffer
// and returns immediately — it never waits for a frame. Do NOT use
// Serial.readBytes() or any other blocking read here: if the sensor drops out, a
// blocking read stalls the control loop. The tvc-drone reference project calls
// this out as a crash cause.
//
// A reading whose last good frame is older than cfg::MTF01_TIMEOUT_MS is marked
// invalid. Consumers must check `valid` every tick.
#pragma once

#include <cstdint>

namespace mtf01 {

struct Reading {
    float    flowX_radps;  // ANGULAR flow rate, not velocity (see fusion/translation)
    float    flowY_radps;
    float    range_m;      // downward range, 0..12 m
    uint32_t lastFrameMs;  // millis() of the last good frame
    bool     valid;        // false before the first frame and after the timeout
};

// Open the UART on cfg::MTF01_RX_PIN / MTF01_TX_PIN. Call once in setup().
bool init();

// Drain and parse buffered bytes, then apply the timeout. Call every control tick.
// Never blocks.
void update();

// Most recent parsed frame, with `valid` already reflecting the timeout.
Reading latest();

}  // namespace mtf01
