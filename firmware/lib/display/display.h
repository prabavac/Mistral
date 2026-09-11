// display.h — on-board OLED — Mistral
//
// U8g2 SSD1306 (128x64) on the Heltec V3's on-board panel. Pins are hardcoded
// from mistral_config.h: cfg::OLED_SDA, OLED_SCL, OLED_RST, OLED_ADDR. cfg::VEXT
// must be driven LOW to power the panel — without it every draw is a silent no-op.
//
// Full-frame I2C writes are slow: never draw at control-loop rate.
#pragma once

namespace display {

// Power the panel (VEXT low), pulse reset, start U8g2. Call once in setup().
void init();

// Clear and draw one or two lines of text.
void message(const char* line1, const char* line2 = nullptr);

}  // namespace display
