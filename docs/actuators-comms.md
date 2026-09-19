# Mistral: Actuators, Comms, and Ground UI

Spec packet for firmware implementation. Read alongside `PROJECT-CONTEXT.md`.

Everything marked **MEASURED** came off the bench and is authoritative. Everything
marked **UNVERIFIED** is inherited from the reference project or a datasheet and must
be confirmed on the stand before flight.

---

## 0. Invariants: violating any of these produces silent failure

1. **`PWM_RES_BITS = 14`.** The ESP32-S3 LEDC timer is 14 bits wide (1–14 for S3; 1–20
   only on the original ESP32). `ledcAttachChannel(pin, 50, 16, ch)` returns `false`
   and **no pulses are produced at all**. The sketch looks alive, the serial log
   prints plausible microseconds, and nothing moves. Always check the return value
   and halt on failure rather than continuing.

2. **Native LEDC only.** Do NOT use ESP32Servo: its shared MCPWM timers collide on
   the S3 and corrupt the ESC into an uncommanded spin. Do NOT use the Dlloydev
   ESP32-AnalogWrite lib (old `ledcSetup` API, won't compile on core 3.x). Do NOT use
   bare `ledcAttach` (auto-channel), which returns fail; use explicit channels. Do NOT
   call `ledcAttachChannel` from `loop()`, where it returns `false`.

3. **Attach each channel exactly once, in `setup()`.**

4. **NO SOFTWARE INTERPOLATION between servo positions.** Command the target pulse
   directly and let the servo slew to it natively. The analog servo's internal
   proportional loop *is* the smooth glide. Stepping through intermediate targets
   makes the servo re-converge on each one and is visibly steppy, confirmed twice on
   this hardware. In flight the controller already produces a smoothly varying target
   each tick; there is nothing to interpolate.

5. **Write on change only.** Re-issuing an identical pulse every loop makes an MG90S
   buzz. Compare the computed duty count against the last one written and skip if
   equal.

6. **Common ground across everything**: both ESCs, the ESP32, all servos, the BEC,
   and all sensors. A floating ground cost a full debugging session on this build: the
   ESCs beeped continuously because they had no reference for the PWM signal and saw
   no throttle at all.

7. **ESC and servo write paths stay separate.** `throttle::*` and `tvc::*`. No shared
   "write all PWM" loop.

---

## 1. TVC servos

### Hardware

| | |
|---|---|
| Servos | 2× MG90S, metal gear |
| Supply | 4.8–6.0 V from the **5 V BEC**, never from the Heltec |
| Current | 10 mA idle, 120–250 mA moving, **up to 700 mA stalled** |
| Dead band | **5 µs**, finer commands are ignored by the servo's comparator |
| Mass | 13.4 g each |
| Signal | 3.3 V direct from the S3. **No level shifter.** |

Metal gear is not a preference. The reference project chose it specifically because
metal gears have far less backlash than nylon, and backlash is the limiting factor on
gimbal precision.

Add **100–470 µF electrolytic** across the servo +5 V and ground, physically close to
the connectors. Absorbs start-up inrush and reduces signal-line jitter.

Never let the gimbal reach a hard stop. A stalled MG90S at 700 mA strips teeth. The
clamp and the per-servo µs backstops exist for this.

### Pin map

| Function | GPIO | LEDC ch |
|---|---|---|
| Servo X (pitch) | **48** | 2 |
| Servo Y (roll) | **5** | 3 |

GPIO 48 is noisy for *continuous* PWM but fine for a servo, which only slews to a held
position. GPIO 5 replaced GPIO 4 on Zephyr after a confirmed wiring fault on GPIO 4,
treat GPIO 4 as suspect on this board family until proven otherwise.

### Calibration: MEASURED

```
X_CENTER = 1626 us
Y_CENTER = 1584 us
SPAN     = 500 us   (+/- about centre)
HARD_MIN = 1000 us  backstop, enforced below the clamp
HARD_MAX = 2250 us
```

X therefore travels 1126–2126, Y travels 1084–2084. Both confirmed clear of mechanical
stops at full ±500.

In firmware each servo's µs backstop is its verified centre ±500 range, which is
tighter than HARD_MIN/HARD_MAX (X's upper stop was only checked to 2126, not 2250).
HARD_MIN/HARD_MAX are kept as absolute limits: `mistral_config.h` fails to compile if
a per-servo backstop is ever set outside them.

Resolution at 14 bits / 50 Hz is **1.22 µs per count**, four times finer than the 5 µs
dead band. There is no resolution problem to solve.

### Gear ratios: 3:1 on X, 4:1 on Y. Corroborated, not yet measured.

The reference project uses **servo-x 3:1, servo-y 4:1**. The asymmetry is kinematic,
not a design choice: servo-y's gear revolves around the TVC axis as it rotates, so it
needs one extra full rotation per 360° of TVC travel.

**Corroborating observation on this gimbal:** at ~10.5 µs per servo degree, ±500 µs is
roughly ±48° of servo travel, which through the two ratios predicts ±15.9° of TVC on X
and ±11.9° on Y, with Y travelling about a quarter less than X. Running the full ±500 sweep
on this hardware, **Y does visibly sweep about a quarter less than X**. The inherited
ratios are therefore consistent with the built gimbal.

**Still required before flight: an absolute protractor measurement.** The sweep confirms
the *ratio between* the axes, not the absolute scale, a common error in
`usPerServoDeg` would shift both axes together and go undetected. Command a known TVC
angle, measure what actually happens, and correct `usPerServoDeg` or `gearRatio` to
match. This number feeds the control-effectiveness constant, so an error here scales
every commanded correction.

Note the reference does NOT encode the ratio in firmware. Their driver is a bare map,
`pulse = 1000 + (perc + offset) × 10` with offsets 25 (x) and 20 (y), pure mechanical
trim. The ratio is absorbed into the control-effectiveness constants they measured on
a bifilar pendulum. We encode the ratio explicitly so commanded angles stay physical
and the clamp means degrees of nozzle, not microseconds.

The bench sweep sketch works purely in microseconds and contains no ratio. The ratio
enters at `lib/tvc`.

### Structure: `lib/tvc`

- `init()`: attach both channels once, park at centre, return false on attach failure
- `setDeflection(float pitchDeg, float rollDeg)`: clamp, apply ratio and sign, write
- `setTrim(axis, us)`: live centre adjustment, rides on top
- `centre()`
- Per-servo `ServoCal` struct in config: centre µs, µs per servo degree, gear ratio,
  direction sign, µs backstops

Because the ratios differ per axis, the TVC clamp must be enforced in **TVC degrees**,
before the ratio is applied, not in microseconds. A single µs clamp would give the two
axes different angular limits.

**Verify the direction sign of every servo on the stand before free flight.** A
reversed sign steers the vehicle into the ground.

---

## 2. ESCs and throttle

### Hardware

| | |
|---|---|
| ESCs | 2× 12 A with BEC |
| Motor | DZP30 contra-rotating, 1500 KV, 32 g |
| Props | GWS 7035 three-blade pair |
| Draw | 5.1 A per motor at 11.1 V, 0.5 A idle |
| Thrust | 480–560 g total, **unresolved**, see PROJECT-CONTEXT §8 |
| Signal | 3.3 V direct. No level shifter. |

No buffer is needed. PX4's documentation states PWM inputs use TTL/CMOS levels, where high
is above 2.0 V, and 5 V levels are never required to switch an input on. The
reference project drives these same ESCs directly from a 3.3 V STM32 with no level
shifter anywhere in its BOM. (Zephyr needed a 74AHCT125N only because its specific 80 A
ESC refused to arm on 3.3 V.)

### Pin map

| Function | GPIO | LEDC ch |
|---|---|---|
| ESC 1 (upper motor) | **6** | 0 |
| ESC 2 (lower motor) | **7** | 1 |

**Two independent channels. Never Y-split.** Differential RPM between the two motors
is the yaw actuator on this airframe: the reference's control vector is
`[servo-x, servo-y, differential thrust, main thrust]`. Y-splitting removes an axis of
control.

**10 kΩ pulldown from each signal pin to ground.** GPIOs float during MCU boot, and an
ESC seeing a floating or high throttle line will either enter safety lockout or read it
as a full-throttle arming command. With the pulldown the line reads cleanly low, so the
ESC sees no pulses rather than garbage.

### Throttle range calibration: timing is critical

Standard procedure, but there is a **~4 second window** that is easy to miss:

1. Both outputs at MIN. Battery disconnected. Props off.
2. Both outputs to **MAX (2000 µs)**.
3. Connect the battery. ~2 s later the ESC emits **beep-beep** confirming the high point.
4. **Within about 4 seconds**, drop to **MIN (1000 µs)**. Hold ~3 s. Beep-beep confirms
   the low point, then the self-detect tone.

Miss the window and the ESC continues into its **programming menu** instead, a
5-second pause followed by groups of tones. If you hear that, unplug, replug, retry.

Working values on this hardware: 5 s to unplug, MAX held for 3 s with a "PLUG IN NOW"
countdown, then MIN for 4 s. Fully automatic, no keypresses.

Symptom of an uncalibrated ESC: won't spin below roughly 1610 µs. Recalibrating fixes
it with no power cycle needed.

### Diagnosing ESC beeps

Two alarms, distinguished by spacing:

- **Single beeps ~2 s apart** → throttle signal abnormal or not detected. Check common
  ground first, then that pulses are actually present, then wire order.
- **Paired beeps ~1 s apart** → input voltage outside the accepted range.

Meter check for "are pulses present": DC volts on the signal pin against ground. At
1000 µs in a 20000 µs period that's 5% duty, so the meter averages about **0.16 V**. At
2000 µs it reads about **0.33 V**. Zero means no output; a steady 3.3 V means stuck high.

### Throttle scaling

- Range 1000–2000 µs.
- **Arming:** hold MIN for ~3 s before accepting any throttle. Software gate: forced to
  ESC min unless armed, kept as defence in depth even though the LEDC channels isolate
  the outputs in hardware.
- **Thrust goes as RPM², so voltage compensation is squared, not linear.** Correct the
  commanded pulse by `(V_nominal / V_measured)²`.
- **There is currently no voltage sense on this build.** The reference runs an INA226
  configured for 140 µs conversions specifically so telemetry doesn't bottleneck its
  100 Hz loop, and uses it for exactly this compensation. Until that's added, hang an
  external cell-checker alarm on a balance lead. At hover current, full to floor is
  about five minutes.
- **Auto-cut:** kill throttle after a configurable idle period at non-zero throttle.
  Prevents an unattended bench rig running a pack flat.

### Structure: `lib/throttle`

`init()`, `arm()`, `disarm()`, `setNormalised(float 0..1)`, `calibrate()`, and
`setDifferential(float)` for the yaw channel. Differential applies ±delta to the two
ESCs around a common base: positive yaw adds to motor 1 and subtracts equally from
motor 2.

---

## 3. WiFi: SoftAP (`lib/wifi_link`)

### Configuration

- **SoftAP mode**, not station. The vehicle is the access point; the phone or laptop
  joins it. No router dependency in a field.
- `WiFi.softAP(ssid, password)` then `WiFi.softAPIP()`. The default gateway is 192.168.4.1.
- SSID `mistral-<last4 of MAC>`, WPA2 with a fixed password in config.
- **WiFi runs on core 0, the control loop on core 1.** This is a Zephyr convention and
  it matters: without it, a browser connecting can add latency to the attitude loop.

### Critical rule

**The control loop must never block on the network.** Launch, arm, throttle, and gain
edits are *flags and values the loop polls*, never callbacks that do work inline. The
web handler writes to a shared struct; the loop reads it at its own rate. Same
discipline as the MTF-01P UART: no blocking reads in the control path.

### Library

**ESP32Async/ESPAsyncWebServer** plus **ESP32Async/AsyncTCP**, the actively maintained
fork (the original me-no-dev repo is stale). Async matters here because a synchronous
`WebServer` blocks while serving. In `platformio.ini`:

```ini
lib_deps =
    esp32async/AsyncTCP
    esp32async/ESPAsyncWebServer
    bblanchon/ArduinoJson
```

Use **WebSocket**, not polling. A persistent bidirectional connection means telemetry
pushes at whatever rate we choose and every open tab stays in sync. With plain HTTP
requests, state doesn't update across tabs without a refresh.

Suggested rates: telemetry push at 10–20 Hz (not the full 100 Hz loop rate, since that
floods the socket for no visible benefit), commands event-driven.

### Serve the page from flash

The page source is `web/index.html` at the repo root, its only copy. `platformio.ini`
embeds it into the firmware image at build time (`board_build.embed_txtfiles`), and
`wifi_link` serves it straight from flash. It is never rebuilt per request, and there is
no hand-maintained second copy to drift (Zephyr mirrored its page into a `PAGE[]` string
and had to keep the two in sync by hand).

The page is one self-contained file with no CDN and no external assets, so it works with no
internet. It can also be opened directly from disk while joined to the AP: loaded over
`file:`, it connects to `ws://192.168.4.1/ws`. WebSocket connections are not subject to
CORS, so unlike Zephyr's HTTP endpoints no `Access-Control-Allow-Origin` header is needed.

The page's transport layer is in place: WebSocket connect with auto-reconnect, a
LINK / STALE / NO LINK indicator, one handler per frame `type`, and a `send()` that tags
every command with an `id` and matches the ack (timeout shown as a failure). Panels plug
into that. Rules (the first three carried over from Zephyr's page):

- **Arm, disarm and kill zero the throttle on the vehicle.** `wifi_link` resets the held
  throttle to 0 when it receives any of them, so each flight starts from zero and a stale
  slider value can't block re-arming. `main.cpp` applies the throttle level before
  `arm()` so the arm sees that zero. The page drops any throttle value still waiting to
  be sent when it sends one of these commands.
- **Sliders are coalesced.** `sendLevel()` sends the first value at once, then only the
  newest value every 60 ms while the slider moves, and always ends on the final value.
- **Settings are remembered, hardware facts are not.** Keys listed in the page's
  `PERSIST` map (live gains, once they exist) are saved in the browser and re-sent on
  the first telemetry frame after the page loads and whenever the vehicle reboots.
  Throttle and arm state are never saved (restoring them could spin a motor). Trims,
  servo signs and axis mappings are never saved either: they belong in
  `mistral_config.h`, and a remembered trim re-sent after its value was copied into the
  config centre would apply the offset twice.
- **Link loss disarms.** The page pings every 250 ms, but not while it is hidden. The
  vehicle disarms when no frame has arrived from any client for `LINK_TIMEOUT_MS`
  (1 s). Hiding the page (switching apps, locking the phone) also sends a disarm.

---

## 4. Ground station web UI

**Built so far (bench bring-up):** status bar, KILL, arm/disarm, throttle, and a nozzle
readout for the bench sweep. Everything else in this section is still design only; it
is here so the WebSocket message schema is designed with the final UI in mind.

### Layout

Single page, dark, phone-first, since it'll be used outdoors on a phone one-handed. No
frameworks, no CDN, no build step. One self-contained HTML file with inline CSS and JS.

### Panels

**1. Status bar (always visible, top)**
State (DISARMED / ARMED / FLYING / KILLED), link quality, uptime, pack voltage when
sensing exists. Colour-coded: grey disarmed, amber armed, green flying, red killed.

**2. KILL button (always visible, large, red, top-right)**
Instantly cuts throttle and disarms. Must be reachable without scrolling from any
panel. This is the single most important control on the page.

**3. Arm / disarm**
Two-step: a toggle that requires a confirm tap. Shows the arming countdown.

**4. Throttle**
Large vertical slider, 0–100%, plus the resulting microseconds as text. Disabled and
visually greyed unless armed. Snap-to-zero on release is configurable (default on).
Dragging is relative to where the finger lands, so a tap near the top can never jump
straight to high throttle.

**5. Attitude**
Live pitch/roll/yaw as numbers and a simple artificial-horizon SVG. Commanded nozzle
deflection per axis shown alongside actual, so lag is visible. Because X and Y have
different gear ratios, display TVC degrees, not microseconds, because the two axes are not
comparable in µs.

**6. Trim**
Per-servo centre adjustment in microseconds, ± buttons with a live readout, and a
"copy to clipboard as config" button that emits the four constants ready to paste into
`mistral_config.h`. This is how bench calibration numbers get out of the vehicle.

**7. Gains**
Live-editable PID and LQR gains per axis. Edits apply immediately, which is the whole
point, since gains are tuned on a stand with the vehicle running. A "revert to
compiled defaults" button.

**8. Console**
Scrolling log of firmware messages, same stream as serial. Lets you debug in the field
without a laptop.

### Message schema

JSON both directions. Telemetry frames tagged by type so the UI can route them without
parsing everything. Commands acknowledged so the UI can show a failed command rather
than silently dropping it.

The implemented schema (command types, acks and the telemetry frame) is documented
at the top of `firmware/lib/wifi_link/wifi_link.h`. That header is the reference; extend
it there as panels gain firmware support.

### Non-goals

No flight planning, no maps, no logging UI, no multi-vehicle. Bench tuning and manual
hops only.
