# Mistral — bench verification

Run these in order; each one gates the next. **Props off throughout.** Restrain the
airframe for anything past step 2.

Firmware controls are on the ground page (join `mistral-xxxx`, open
http://192.168.4.1): **ARM** (two taps) → servos live, motors off · **FLY** (two taps) →
ESCs arm, throttle live, integrators on · **MOTORS OFF** (the ARM button while flying) →
back to ARMED · **DISARM / KILL** → DISARMED, servos centred.

Serial (115200) prints one line every 200 ms:

```
STATE     r <r1> <r2>  dr <dr1> <dr2>  i <ir1> <ir2>  u <u1> <u2>  us <X> <Y>
          deg          deg/s            rad·s          deg           µs
```

> Do **not** enable `THRUST_SCHED`, raise `LQR_GAIN_SCALE` above 0.3, or widen
> `TVC_CLAMP_RAD` without the owner's explicit say-so. Those three are the difference
> between a bench test and a crash.

## 1. Gyro bias at boot

- Power up upright and perfectly still. Serial prints `IMU ok  gyro bias x y z dps`;
  each axis should be well under 0.1 dps.
- At rest, `dr` sits near 0 and `r` does not creep.

Observed 2026-09-13 on serial: −0.011 / +0.001 / −0.003 dps, angles steady.

## 2. ARMED — each servo opposes its tilt

1. Tap **ARM** twice. The pill reads ARMED; `i` must read 0.000 / 0.000 the whole time.
2. Tilt about **body X** by hand. **Servo X** (GPIO 48) moves and servo Y doesn't. The
   nozzle must deflect to **oppose** the tilt.
3. Tilt about **body Y**. **Servo Y** (GPIO 5) moves and servo X doesn't. It must oppose.
4. Tilt far enough to saturate: u stops at ±8.59°, which is X 1956 / 1296 µs and
   Y 1144 / 2024 µs (12.8 µs per servo degree).

If a servo **helps** the tilt, flip its `dir` in `SERVO_X_CAL` / `SERVO_Y_CAL`
(`mistral_config.h`), reflash, repeat. If the **wrong servo** responds, the axis pairing
is wrong: stop and report.

Record: `dir` X = ____, `dir` Y = ____ (config ships +1 / −1 from the bench sketch).

## 2b. Gear ratios — measure, don't inherit

With `BENCH_SERVO_SWEEP = true`, DISARMED steps each nozzle in turn: X +8.59°, X −8.59°,
Y +8.59°, Y −8.59°, each held 5 s (the Nozzle panel shows which). Measure each nozzle's
**full swing** between its + and − positions with a protractor or phone inclinometer. The
command is 17.19°; using the full swing cancels any zero offset.

    true ratio = config ratio × 17.19° ÷ measured swing

**Ratios confirmed 2026-09-13: X 3:1, Y 4:1.** This build uses the reference's printed
gimbal and servos unchanged; tvc-drone's `docs/en/Hardware.md` states 3:1 / 4:1, and its
firmware servo scales (220.4 / 293.3 %/rad) are in the same 4:3 proportion. Keep this step
for the shared `usPerServoDeg` scale: if both axes swing short or long by the same factor,
that is what's wrong. `usPerServoDeg` is 12.8, adopted from the reference's constants on
2026-09-13 in place of an unmeasured 10.5; a 17.19° full swing on both axes confirms it.
Turn `BENCH_SERVO_SWEEP` on to run this step (it ships off).

Record: X swing ____°, Y swing ____°.

## 3. Fusion vs complementary filter on the same tilts

The Attitude panel shows both side by side. On every tilt they must match in sign, be
close in size, and both return to ~0 when level. A disagreement means one of them is
wrong: do not proceed.

Owner confirmed 2026-09-13, before the controller was wired. Repeat if the IMU mount
changes. `ATTITUDE_USE_FUSION` picks the controller's input (currently Fusion).

## 4. FLYING — integrals accumulate only at ≥ 50 % throttle

**Props off and airframe restrained: this step spins the motors.**

1. From ARMED, tap **FLY** twice. ESCs hold at minimum for ~3 s (countdown), then the pill
   reads FLYING and the note says integrators **held at zero**.
2. Hold a small steady tilt and raise the throttle to below 50 %: `i` stays 0.000. (Untick
   *Snap to zero on release* if you want the slider to stay put.)
3. Raise it to 50 % or more: the note says integrators **on**, and `i` on the tilted axis
   grows and stops at ±0.400. If u is saturated, it stops growing in the direction that
   would push further.
   **Setting it back down level does NOT unwind the integral, and that is expected.** The
   loop is open on the bench: the servos move but the vehicle doesn't rotate, so the
   error never reverses. The integral holds (or keeps creeping from any small residual
   angle) and the nozzle stays offset by about −0.3 × 0.294 × i rad (~2° at i = 0.4).
4. Drop the throttle below 50 %: `i` returns to 0.000. (MOTORS OFF, DISARM and KILL also
   zero it; that path is host-tested.)
5. **DISARM**: servos centre at X 1626 µs, Y 1584 µs, or run the 2b sweep if it's enabled.

`LQR_INTEGRATE_MIN_THROTTLE` (50 %) is a placeholder until thrust is measured: set it just
under lift-off throttle.

## 5. Only then

Thrust scheduling, and the scale test for actual thrust (480–560 g is unresolved).
Scheduling needs a measured throttle → newtons curve before `THRUST_SCHED` means anything.

## Not in this firmware

- Yaw control: the differential stays 0; yaw rate is displayed only.
- Link lost for 1 s, or the page hidden: → DISARMED.
- Throttle unchanged for 60 s while FLYING: auto-cut → ARMED.
