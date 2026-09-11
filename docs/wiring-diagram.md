# Mistral — Wiring Diagram

> Pin numbers mirror `firmware/include/mistral_config.h`. If the two disagree,
> the config header wins — fix this doc.

## Pin map (Heltec WiFi LoRa 32 V3, ESP32-S3)

| GPIO | Function | Notes |
|------|----------|-------|
| 6    | ESC1 signal | LEDC channel 0 |
| 7    | ESC2 signal | LEDC channel 1 — separate wire, never Y-split |
| 48   | TVC servo X | LEDC channel 2, 3:1 gear |
| 5    | TVC servo Y | LEDC channel 3, 4:1 gear |
| 41   | I2C SDA (sensor bus) | ICM-42688 @ 0x68 (bring-up), BMP388 @ 0x76/0x77 |
| 42   | I2C SCL (sensor bus) | |
| 17   | OLED SDA | on-board SSD1306 @ 0x3C |
| 18   | OLED SCL | |
| 21   | OLED RST | |
| 36   | VEXT | drive LOW to power the OLED |
| TBD  | MTF-01P UART RX / TX | 115200 baud |
| TBD  | ICM-42688 SPI | preferred flight interface |

## Power architecture

TBD — battery, ESC BECs, servo 5 V rail, 3.3 V logic.

## ESC signal level

Zephyr's ESC would not arm on a 3.3 V signal and needed a 74AHCT125N buffer.
Check whether Mistral's ESCs accept 3.3 V before wiring them direct.

## Grounding

Common ground across battery, both ESCs, servos, Heltec and every sensor. A
floating ground is the most common first-build failure.
