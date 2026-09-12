# Mistral

Thrust-vectored coaxial contra-rotating drone on a Heltec WiFi LoRa 32 V3 (ESP32-S3); successor to Zephyr.
**Spec: [`PROJECT-CONTEXT.md`](PROJECT-CONTEXT.md) (authoritative).** Details in [`docs/`](docs/). This file points; it does not duplicate.

## Authorship — Claude is never a co-author
Claude must NOT add itself as a co-author of this code. No `Co-Authored-By: Claude` trailer and no
"Generated with Claude Code" line in any commit or PR. The repository owner is the ONLY author of every commit.
Do not commit or push unless explicitly asked.

## Build & flash
```bash
pio run                  # build — must be clean before any commit
pio run -t upload        # flash over USB
pio device monitor       # 115200; DTR/RTS held off so connecting doesn't reset the board
```
`platformio.ini` (repo root) remaps src/lib/include into `firmware/`. Platform is pinned to pioarduino
(core 3.x) — stock `espressif32` ships core 2.x, which lacks the LEDC API below.

## Code rules
- **All hardware constants live in `firmware/include/mistral_config.h`.** Nothing magic in module logic.
- One subsystem = one library under `firmware/lib/`. `main.cpp` only wires modules in `setup()`/`loop()`.
- Conventional Commits: `type(scope): summary`, scope = module name (`tvc`, `throttle`, `mtf01`, …).

## LEDC — do not regress (each of these failed on Zephyr)
- Native LEDC only: `ledcAttachChannel(pin, 50, 14, ch)`, EXPLICIT channel, called ONCE in the module's `init()`.
- `ledcWrite()` takes the PIN on core 3.x. `usToDuty = (us << 14) / 20000`.
- NO ESP32Servo (shared MCPWM timers → uncommanded ESC spin). NO `ledcAttachChannel` in `loop()` (returns
  false). NO `ledcAttach` auto-channel.
- Servos write on change only — re-issuing the same pulse makes the MG90S buzz.

## Safety invariants
- TVC clamp (`TVC_CLAMP_DEG`) enforced in `lib/tvc` for EVERY servo, with a per-servo µs backstop behind it.
- Throttle software-gated behind arming: both ESCs forced to `ESC_MIN_US` unless armed.
- The two ESC channels stay separate — never Y-split, no shared "write all PWM" loop.
- Saturation flags propagate back up the control chain to freeze upstream integrators.
- DISARMED: control loops bypassed, integrators held at zero.
- `GEAR_RATIO_Y = 4.0` is intentional (see its config comment) — don't "fix" it.
- Every servo sign is verified on the stand before free flight.
