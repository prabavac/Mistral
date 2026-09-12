# Mistral — Wiring Diagram

> Pin numbers mirror `firmware/include/mistral_config.h`. If the two disagree,
> the config header wins — fix this doc.

## Pin map (Heltec WiFi LoRa 32 V3, ESP32-S3)

| GPIO | Function | Notes |
|------|----------|-------|
| 6    | ESC1 signal (upper motor) | LEDC channel 0, 10 kΩ pulldown to GND |
| 7    | ESC2 signal (lower motor) | LEDC channel 1, 10 kΩ pulldown to GND — separate wire, never Y-split |
| 48   | TVC servo X (pitch) | LEDC channel 2, 3:1 gear |
| 5    | TVC servo Y (roll) | LEDC channel 3, 4:1 gear — not GPIO 4 (wiring fault on Zephyr) |
| 41   | I2C SDA (sensor bus) | ICM-42688 @ 0x68 (bring-up), BMP388 @ 0x76/0x77 |
| 42   | I2C SCL (sensor bus) | |
| 17   | OLED SDA | on-board SSD1306 @ 0x3C |
| 18   | OLED SCL | |
| 21   | OLED RST | |
| 36   | VEXT | drive LOW to power the OLED |
| TBD  | MTF-01P UART RX / TX | 115200 baud |
| TBD  | ICM-42688 SPI | preferred flight interface |

## Power architecture

- **ESCs:** 2× 12 A with BEC. Each motor draws 5.1 A at 11.1 V, 0.5 A idle.
- **Servo rail:** 4.8–6.0 V from the 5 V BEC — never from the Heltec. Each MG90S
  draws up to 700 mA stalled. Put a 100–470 µF electrolytic across servo +5 V and
  GND, close to the connectors.
- **Voltage sense:** none yet. Hang a cell-checker alarm on a balance lead.
- Battery and 3.3 V logic supply: TBD.

## ESC signal level

3.3 V direct from the S3 — no level shifter. The ESCs read TTL/CMOS levels (high
above 2.0 V), and the reference project drives the same ESCs from a 3.3 V STM32.
Zephyr's 74AHCT125N buffer was needed only for its specific 80 A ESC.

## Grounding

Common ground across battery, both ESCs, servos, BEC, Heltec and every sensor. A
floating ground is the most common first-build failure: on this build it left the
ESCs beeping single beeps ~2 s apart, seeing no throttle signal at all.

See [`actuators-comms.md`](actuators-comms.md) for ESC calibration and beep codes.
