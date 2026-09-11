# Mistral
Coaxial propellor-powered UAV. Another proff-of-concept of SPARC, loosely modeled off of Zephyr. Rocket form factor, GNC learning.

**Thrust-vectored coaxial contra-rotating drone.** Two counter-rotating motors
lift the vehicle and cancel each other's torque. A two-servo gimbal vectors the
thrust for pitch and roll, and differential motor thrust controls yaw. Successor
to Zephyr; the mechanical and control approach follows
[fdiwth/tvc-drone](https://github.com/fdiwth/tvc-drone) (architecture only — no
code is borrowed).

> **Status: scaffold.** The repository layout, build system and module
> interfaces are in place. Every module is a stub; sensor drivers, estimation,
> control and the main loop are written one module at a time against real
> hardware.

The full vehicle spec is [`PROJECT-CONTEXT.md`](PROJECT-CONTEXT.md).

## Authorship

**Claude (or any AI assistant) may not be listed as a co-author of this code.**
When code is committed, the only author is the repository owner — no
`Co-Authored-By` trailers or AI attribution lines in commits or pull requests.

## Hardware

| Subsystem | Component |
|-----------|-----------|
| Flight computer | Heltec WiFi LoRa 32 V3 (ESP32-S3) |
| IMU | ICM-42688-P |
| Barometer | BMP388 — display only |
| Optical flow + range | MicoAir MTF-01P (UART) |
| Propulsion | 2× DZP30 motors, contra-rotating, 2 independent ESCs |
| TVC | 2× MG90S servos, gear-reduced (X 3:1, Y 4:1) |
| Display | On-board SSD1306 OLED |

## Project structure

```
mistral/
├── platformio.ini          build config (remaps into firmware/)
├── PROJECT-CONTEXT.md      authoritative vehicle spec
├── firmware/
│   ├── include/            mistral_config.h — every pin, limit and gain
│   ├── src/                main.cpp — wiring only
│   └── lib/                sensors (icm42688, bmp388, mtf01), fusion (attitude,
│                           translation), control (lqr), tvc, motors,
│                           display, state_machine, safety
└── docs/                   wiring diagram, flight logs
```

## Build & flash

Requires [PlatformIO](https://platformio.org/). The platform is pinned to the
pioarduino fork (Arduino-ESP32 core 3.x); PlatformIO fetches it on first build.

```bash
pio run                  # build
pio run -t upload        # flash over USB
pio device monitor       # serial monitor, 115200
```

## Safety

- Verify every TVC servo sign on the test stand before any free flight.
- Bench-test motors with props off or the vehicle restrained.
- Throttle is software-gated behind arming; both ESCs sit at minimum otherwise.

## License

See [`LICENSE`](LICENSE).
