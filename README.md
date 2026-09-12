# Mistral
Coaxial propellor-powered UAV. Another proff-of-concept of SPARC, loosely modeled off of Zephyr. Rocket form factor, GNC learning.

**Thrust-vectored coaxial contra-rotating drone.** Two counter-rotating motors
lift the vehicle and cancel each other's torque. A two-servo gimbal vectors the
thrust for pitch and roll, and differential motor thrust controls yaw. Successor
to Zephyr; the mechanical and control approach follows
[fdiwth/tvc-drone](https://github.com/fdiwth/tvc-drone) (architecture only — no
code is borrowed).

> **Status: bring-up.** The actuators (`tvc`, `throttle`) and the WiFi ground
> link (`wifi_link`) are implemented. The gimbal holds at centre (a diagonal
> bench sweep is available via `cfg::BENCH_SERVO_SWEEP`) and the throttle is
> driven from the ground station page. Sensor drivers, estimation, control and the state machine are
> still stubs, written one module at a time against real hardware.

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
│                           translation), control (lqr), tvc, throttle,
│                           display, state_machine, safety, wifi_link, bench
├── web/                    index.html — ground-station page, embedded into the firmware
└── docs/                   wiring diagram, actuators & comms, flight logs
```

## Build & flash

Requires [PlatformIO](https://platformio.org/). The platform is pinned to the
pioarduino fork (Arduino-ESP32 core 3.x); PlatformIO fetches it on first build.

```bash
pio run                  # build
pio run -t upload        # flash over USB
pio device monitor       # serial monitor, 115200
```

## Ground station page

The vehicle hosts its own WiFi access point and serves the ground-station page:

1. Join the AP **`mistral-xxxx`** — the last four hex digits of the MAC, printed on
   serial at boot. The password is `cfg::WIFI_PASSWORD`.
2. Open **http://192.168.4.1**.

The page source is [`web/index.html`](web/index.html). The build embeds it into the
firmware, so edit that file and reflash — there is no second copy to keep in sync.
While joined to the AP you can also open the file straight from disk; it connects to
192.168.4.1. It has the status bar, KILL, two-tap arm/disarm, the throttle slider and a
nozzle readout; the remaining panels are specified in
[`docs/actuators-comms.md`](docs/actuators-comms.md) §4.

## Safety

- Verify every TVC servo sign on the test stand before any free flight.
- Bench-test motors with props off or the vehicle restrained.
- Throttle is software-gated behind arming; both ESCs sit at minimum otherwise.
- Losing the ground link for 1 s disarms. So does KILL, and hiding the page
  (switching apps or locking the phone).

## License

See [`LICENSE`](LICENSE).
