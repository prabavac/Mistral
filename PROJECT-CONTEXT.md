# Mistral — Project Context

**This is the authoritative spec for the vehicle.** When code, docs, or CLAUDE.md
disagree with this file, this file wins — fix the other one. Hardware numbers
live in `firmware/include/mistral_config.h`; this file states the rules they obey.

## 1. Vehicle

Mistral is a thrust-vectored coaxial contra-rotating drone in a rocket form
factor, built for GNC learning. It succeeds Zephyr (dual side-mount EDF).

- **Lift:** two coaxial, counter-rotating DZP30 motors. Counter-rotation cancels
  reaction torque.
- **Pitch / roll:** thrust vectoring through a two-servo gimbal.
- **Yaw:** differential thrust between the two motors. There is no yaw servo.

## 2. Reference design

[fdiwth/tvc-drone](https://github.com/fdiwth/tvc-drone) (STM32F722, same DZP30
motor) is the reference for the mechanical and control approach. Borrow its
**architecture, not its code.** Relevant ideas taken from it:

- Gear-reduced MG90S gimbal with 3:1 on X and 4:1 on Y (section 5).
- LQR attitude control.
- Anti-windup by propagating actuator saturation back up the control chain.
- A flight state machine that holds integrators at zero while disarmed.
- Flow sensor reads treated as a known crash risk if they block.

## 3. Hardware

| Role | Part | Interface | Rule |
|------|------|-----------|------|
| Flight computer | Heltec WiFi LoRa 32 V3 (ESP32-S3) | — | Arduino-ESP32 core 3.x |
| IMU | ICM-42688-P | I2C 0x68 for bring-up; SPI preferred | driver library chosen at bring-up |
| Barometer | BMP388 | I2C 0x76 / 0x77 | **display only — never altitude feedback** |
| Flow + range | MicoAir MTF-01P | UART 115200 | optical flow + 12 m rangefinder in one module |
| Motors | 2× DZP30 | 2 independent ESCs | never Y-split |
| TVC | 2× MG90S, gear-reduced | LEDC PWM | clamp + µs backstop per servo |
| Display | on-board SSD1306 | U8g2, hardcoded pins | — |

## 4. Estimation

- **Attitude** (`fusion/attitude`): IMU → pitch, roll, yaw **rate**. There is no
  magnetometer on this build, so yaw is rate only — no heading.
- **Translation** (`fusion/translation`): flow + range → lateral velocity and
  position.
  - Flow is **angular**: `actual_velocity = reported_flow × altitude`.
  - Range is a **scale factor only**, never vertical feedback. It multiplies the
    whole estimate, so it is low-passed before use.
- **Barometer:** display only.
- **MTF-01P reads are non-blocking.** Use an explicit timeout and a validity flag.
  A blocking `Serial.readBytes()` stalls the control loop when the sensor drops
  out, which the reference project documents as a crash cause.

## 5. Control & actuation

- **Attitude controller:** LQR only (`control/lqr`). There is no PID.
- **State machine:** DISARMED / ARMED / FLYING / LANDED / KILL. While DISARMED,
  control loops are bypassed and integrators are held at zero, so nothing winds up
  on the ground.
- **Anti-windup:** saturation flags propagate **back up** the control chain and
  freeze upstream integrators.
- **TVC** (`tvc`): the interface takes **TVC degrees** and applies the gear ratio
  internally. Each servo has a `ServoCal` with `centerUs`, `usPerServoDeg`,
  `gearRatio`, `dir`, and a `minUs`/`maxUs` backstop.
  - `GEAR_RATIO_X = 3.0`.
  - `GEAR_RATIO_Y = 4.0`, **on purpose**. Servo-y's gear revolves around the TVC
    axis as it rotates, so it needs one extra full rotation per 360° of TVC travel.
- **Motors** (`motors`): two independent ESC channels, commanded as
  `setThrust(common, differential)`.
- **Control loop:** 100 Hz.

### PWM conventions — do not deviate

- Native LEDC only. Call `ledcAttachChannel(pin, 50, 14, ch)` with **explicit**
  channel numbers, **once**, in each module's `init()`.
- `ledcWrite()` takes the **pin** on core 3.x. `usToDuty = (us << 14) / 20000`.
- Channels: ESC1 → 0, ESC2 → 1, servo X → 2, servo Y → 3.
- **Never** use ESP32Servo: its shared MCPWM timers collide on the S3 and caused
  uncommanded ESC spin on Zephyr.
- **Never** call `ledcAttachChannel` in `loop()` (it returns false).
- **Never** use `ledcAttach` auto-channel.
- Servos are **write-on-change only**; re-issuing the same pulse makes the MG90S buzz.

## 6. Safety invariants

1. The TVC deflection clamp is enforced in `tvc` for **every** servo, with a
   per-servo hard microsecond backstop behind it.
2. Throttle is software-gated behind arming: forced to ESC minimum unless armed.
3. The two ESC channels stay separate. There is no shared "write all PWM" loop.
4. Saturation flags propagate back up the control chain to freeze upstream
   integrators.
5. Every servo sign is verified on the stand before free flight.

## 7. Repository conventions

- All hardware constants live in `firmware/include/mistral_config.h`. Nothing
  magic is buried in logic.
- One subsystem = one library under `firmware/lib/`. `main.cpp` only wires
  modules together.
- Commits use Conventional Commits, `type(scope): summary`, with a module name as
  the scope. `pio run` must build clean first.
- **Authorship:** the repository owner is the only author of every commit. AI
  assistants are never added as co-authors.

## 8. Open decisions

Resolve these during bring-up and record the answers here.

- [ ] MTF-01P UART pins (placeholders in config).
- [ ] ICM-42688 SPI pins, and which driver library.
- [ ] ESC model, and whether it arms on a 3.3 V signal (Zephyr's needed a
      74AHCT125N buffer).
- [ ] Battery.
- [ ] `ServoCal` values: centers, µs/deg, backstops, and signs (on the stand).
- [ ] `ARM_HOLD_MS` and `MTF01_TIMEOUT_MS` (placeholders in config).
- [ ] LQR state and input vector. The config holds a proposal:
      x = [pitch, roll, pitch rate, roll rate, yaw rate, ∫pitch, ∫roll, ∫yaw rate],
      u = [tvcX, tvcY, differential].
- [ ] Flow rotation compensation. Does the MTF-01P remove body rotation
      internally, or must fusion subtract gyro rates, as tvc-drone does?
- [ ] Which gimbal axis (X/Y) corrects pitch and which corrects roll.
