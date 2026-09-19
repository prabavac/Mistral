// mistral_config.h: Mistral thrust-vectored coaxial drone
//
// SINGLE SOURCE OF TRUTH for every hardware constant: pins, LEDC channels, I2C
// addresses, PWM limits, servo calibration, gains and clamps. Nothing magic is
// buried in module logic. If a number is tied to the airframe, the electronics
// or tuning, it lives here.
//
// Markers:
//   MEASURED    came off the bench, authoritative
//   UNVERIFIED  inherited from the reference or a datasheet; confirm on the stand
//   UNTUNED     placeholder gain; do not fly on it
//   VERIFY      a sign/direction; confirm on the stand before free flight
//   TBD         not chosen yet
//
// Board: Heltec WiFi LoRa 32 V3 (ESP32-S3), Arduino-ESP32 core 3.x.

#pragma once

#include <cstdint>

namespace cfg {

// ─────────────────────────────── Serial ─────────────────────────────────────
constexpr uint32_t SERIAL_BAUD     = 115200;  // matches monitor_speed in platformio.ini
constexpr uint32_t SERIAL_PRINT_MS = 200;     // bench diagnostics line

// ─────────────────────────── I2C sensor bus ─────────────────────────────────
// Sensors are on Wire1, NOT Wire. U8g2's HW_I2C owns Wire (the OLED, GPIO 17/18); sharing
// one bus leaves the sensors addressed on the OLED's pins and the IMU's begin() returns -3.
constexpr uint8_t  I2C_SDA      = 41;  // Wire1
constexpr uint8_t  I2C_SCL      = 42;
constexpr uint32_t I2C_CLOCK_HZ = 400000;

constexpr uint8_t ICM42688_I2C_ADDR = 0x69;  // MEASURED: AD0 pulled high on this breakout
constexpr uint8_t BMP388_I2C_ADDR   = 0x76;  // 0x77 if SDO is pulled high

// ─────────────────────── IMU (ICM-42688-P, lib/sensors/imu) ──────────────────
// From the bench sketch. Mounted rotated 90° about its X axis:
// body X = sensor +X, body Y = sensor +Z, body Z = sensor −Y.
constexpr uint16_t IMU_GYRO_RANGE_DPS = 500;   // Fusion's gyroscopeRange must match
constexpr uint8_t  IMU_ACCEL_RANGE_G  = 8;
constexpr uint32_t IMU_SETTLE_MS      = 1000;  // after configuring, before the bias average
constexpr uint16_t IMU_BIAS_SAMPLES   = 400;   // boot average, vehicle upright and still
constexpr uint8_t  IMU_BIAS_SAMPLE_MS = 3;
// Zero IMU (ground station, DISARMED only) re-runs that average and refuses if the vehicle moves:
// any gyro axis spanning more than this, max − min over the samples. UNTUNED, far above the
// chip's noise behind the 50 Hz UI filter, far below a bump.
constexpr float    IMU_STILL_MAX_SPREAD_DPS = 2.0f;

// Filter the gyro at the SENSOR, never in software: a software LPF adds phase lag inside the
// loop, which the reference identifies as the cause of instability. On-chip UI filter
// (datasheet §5.5): BW index 0 → ODR/2; 1..7 → max(400 Hz, ODR) ÷ 4, 5, 8, 10, 16, 20, 40.
// Order 0 / 1 / 2 = 1st / 2nd / 3rd. The chip resets to index 1 (≈250 Hz at 1 kHz), far above
// the 100 Hz loop's 50 Hz Nyquist, so vibration aliases straight into the rate term.
constexpr uint8_t IMU_GYRO_UI_FILT_BW  = 6;  // 1 kHz ÷ 20 = 50 Hz
constexpr uint8_t IMU_ACCEL_UI_FILT_BW = 6;  // 50 Hz
constexpr uint8_t IMU_UI_FILT_ORDER    = 1;  // 2nd order (the chip's reset order)

// ───────────────────────── Attitude (lib/fusion/attitude) ─────────────────────
// Both estimators always run so they can be compared; this picks the controller's input.
// Fusion, since it matched the bench-verified complementary filter on hand tilts
// (2026-09-13). false switches the controller back to the complementary filter.
constexpr bool  ATTITUDE_USE_FUSION        = true;
constexpr float COMP_FILTER_A              = 0.98f;  // bench sketch, ~0.49 s time constant
constexpr float FUSION_GAIN                = 0.5f;
constexpr float FUSION_ACCEL_REJECTION_DEG = 10.0f;
constexpr float FUSION_REJECTION_TIMEOUT_S = 1.0f;
// Run-time gyro offset tracking, Fusion's FusionBias (owner's call, 2026-09-14). The offset measured
// at calibration drifts as the IMU warms, and Fusion at gain 0.5 holds ~2° of tilt per 1 °/s of it
// (seen after a flight: ≈ −0.46 °/s on both axes). Learns only after every axis has stayed under
// the threshold for the period, and never while FLYING. Fusion's default values.
constexpr float FUSION_BIAS_STILL_DPS = 3.0f;
constexpr float FUSION_BIAS_STILL_S   = 3.0f;

// ─────────────────────── On-board OLED (SSD1306) ────────────────────────────
constexpr uint8_t OLED_SDA  = 17;
constexpr uint8_t OLED_SCL  = 18;
constexpr uint8_t OLED_RST  = 21;
constexpr uint8_t VEXT      = 36;  // OLED power enable, active LOW
constexpr uint8_t OLED_ADDR = 0x3C;

// ─────────────────── MTF-01P optical flow + range (UART) ────────────────────
constexpr uint8_t  PIN_UNASSIGNED   = 0xFF;
constexpr uint8_t  MTF01_RX_PIN     = PIN_UNASSIGNED;  // TBD: ESP32 RX ← sensor TX
constexpr uint8_t  MTF01_TX_PIN     = PIN_UNASSIGNED;  // TBD: ESP32 TX → sensor RX
constexpr uint32_t MTF01_BAUD       = 115200;
constexpr uint32_t MTF01_TIMEOUT_MS = 100;  // TBD: set from the measured frame rate

// ─────────────────────────── LEDC PWM (50 Hz) ───────────────────────────────
// ESCs and servos all run on native LEDC: ledcAttachChannel(pin, PWM_FREQ_HZ,
// PWM_RES_BITS, ch) on the EXPLICIT channels below, ONCE, in the owning module's
// init(). µs → duty: (us << PWM_RES_BITS) / PWM_PERIOD_US.
// PWM_RES_BITS must stay 14: the ESP32-S3 LEDC timer is at most 14 bits, and a wider
// value makes ledcAttachChannel() fail, with no pulses at all. 14 bits at 50 Hz is
// 1.22 µs per count, finer than the MG90S's 5 µs dead band.
constexpr uint32_t PWM_FREQ_HZ   = 50;
constexpr uint8_t  PWM_RES_BITS  = 14;
constexpr uint32_t PWM_PERIOD_US = 20000;

// Each ESC signal pin needs a 10 kΩ pulldown to ground, because GPIOs float during boot.
constexpr uint8_t ESC1_PIN     = 6;  // upper motor
constexpr uint8_t ESC2_PIN     = 7;  // lower motor
constexpr uint8_t ESC1_LEDC_CH = 0;
constexpr uint8_t ESC2_LEDC_CH = 1;

constexpr uint8_t SERVO_X_PIN     = 48;  // pitch
constexpr uint8_t SERVO_Y_PIN     = 5;   // roll, not GPIO 4 (confirmed wiring fault on Zephyr)
constexpr uint8_t SERVO_X_LEDC_CH = 2;
constexpr uint8_t SERVO_Y_LEDC_CH = 3;

// ─────────────────────────────── ESCs ───────────────────────────────────────
constexpr uint16_t ESC_MIN_US  = 1000;  // disarmed / zero thrust
constexpr uint16_t ESC_MAX_US  = 2000;
constexpr uint32_t ARM_HOLD_MS = 3000;  // ESC_MIN hold before throttle is accepted

// Motor balance (owner's call, 2026-09-15): a static ratio that splits the commanded throttle
// between the two rotors so their torques cancel and the airframe stops yawing. ESC1 gets
// throttle x ratio, ESC2 gets throttle x (2 - ratio), so the average is unchanged. 1.0 = equal.
// Clamped in throttle::setBalance; the ground station sends it live. Not a yaw controller: the
// LQR differential stays separate and is still 0.
constexpr float ESC_BALANCE_DEF      = 1.0f;
constexpr float ESC_BALANCE_RANGE[2] = {0.9f, 1.1f};

// Auto-cut: disarm when throttle is non-zero and unchanged for this long, so an
// unattended bench rig can't run the pack flat.
constexpr uint32_t THROTTLE_AUTOCUT_MS = 60000;  // TBD

// throttle::calibrate() timing: MEASURED working values. After MAX, the ESC accepts
// the low point only within ~4 s of battery connect; miss it and it enters its
// programming menu instead.
constexpr uint32_t ESC_CAL_UNPLUG_MS = 5000;  // at MIN: unplug the battery
constexpr uint32_t ESC_CAL_MAX_MS    = 3000;  // at MAX: plug the battery in
constexpr uint32_t ESC_CAL_MIN_MS    = 4000;  // at MIN: low point, then self-detect tone

// ──────────────────────────── Control loop ──────────────────────────────────
constexpr uint16_t CONTROL_LOOP_HZ = 100;

// ──────────────────────────────── TVC ───────────────────────────────────────
// Nozzle deflection clamp, in RADIANS. PRIMARY limit, enforced in lib/tvc for EVERY servo,
// and the LQI's output limit. Clamping nozzle angle rather than µs gives both axes the same
// angular limit even though their gear ratios differ. The bench sketch's U_LIMIT (8.59°).
// Do NOT widen without the user's explicit say-so.
constexpr float TVC_CLAMP_RAD = 0.15f;

// Servo writes: at most one per 50 Hz PWM frame, and only when the target has moved at least
// this far from the last pulse written, which is the MG90S's own dead band. Re-issuing an identical
// pulse makes it buzz; sub-dead-band changes make it dither.
constexpr float SERVO_DEADBAND_US = 5.0f;

// Gear reduction: servo degrees per TVC degree. This build uses the reference's printed
// gimbal, servos and motor unchanged. Its docs/en/Hardware.md states 3:1 (x) and 4:1 (y),
// and its firmware's servo scales (LQR_SXSS 220.4 / LQR_SYSS 293.3 %/rad) are in the same
// 4:3 proportion. A ±500 µs sweep here moves Y about a quarter less than X, consistent.
// Confirmed by the owner 2026-09-13.
constexpr float GEAR_RATIO_X = 3.0f;
// GEAR_RATIO_Y is 4.0, NOT 3.0, deliberately. Do not "fix" it to match X.
// Servo-y's gear revolves around the TVC axis as the gimbal rotates, so the servo
// needs one extra full rotation per 360° of TVC travel: 3 + 1 = 4.
constexpr float GEAR_RATIO_Y = 4.0f;

// Per-servo calibration. lib/tvc maps a clamped nozzle angle to a pulse:
//   us = centreUs + trim + deg(nozzle) · gearRatio · dir · usPerServoDeg
// then clamps us to [minUs, maxUs], the hard backstop behind TVC_CLAMP_RAD.
struct ServoCal {
    uint16_t centreUs;       // pulse at 0° TVC
    float    usPerServoDeg;  // µs per degree at the servo horn, before gearing
    float    gearRatio;      // servo degrees per TVC degree
    int8_t   dir;            // +1 or -1
    uint16_t minUs;          // hard backstop
    uint16_t maxUs;          // hard backstop
};

// MEASURED: centres, and ±SERVO_SPAN_US about each centre is clear of the mechanical
// stops (X 1126–2126, Y 1084–2084). The backstops are exactly that verified range:
// a stalled MG90S draws ~700 mA and strips teeth, so never widen past what was checked.
constexpr uint16_t SERVO_X_CENTRE_US = 1626;
constexpr uint16_t SERVO_Y_CENTRE_US = 1584;
constexpr uint16_t SERVO_SPAN_US     = 500;

// MEASURED absolute pulse limits for either servo. Every per-servo backstop must sit
// inside them, checked at compile time below.
constexpr uint16_t SERVO_HARD_MIN_US = 1000;
constexpr uint16_t SERVO_HARD_MAX_US = 2250;

// usPerServoDeg 12.8, adopted 2026-09-13 from the reference (same MG90S and printed gimbal):
// its LQR_SXSS 220.4 / LQR_SYSS 293.3 %/rad at 10 µs per % are 2204 / 2933 µs per nozzle
// radian, or 12.8 µs per servo degree through 3:1 and 4:1. Replaces an unmeasured 10.5.
// dir: X +1, Y −1, both confirmed by hand-tilting with the bench sketch; re-verify on the
// stand (bench/VERIFY.md). At the clamp X needs ±330 µs and Y ±440 µs, inside ±500.
constexpr ServoCal SERVO_X_CAL = {SERVO_X_CENTRE_US, 12.8f, GEAR_RATIO_X, +1,
                                  SERVO_X_CENTRE_US - SERVO_SPAN_US,
                                  SERVO_X_CENTRE_US + SERVO_SPAN_US};
constexpr ServoCal SERVO_Y_CAL = {SERVO_Y_CENTRE_US, 12.8f, GEAR_RATIO_Y, -1,
                                  SERVO_Y_CENTRE_US - SERVO_SPAN_US,
                                  SERVO_Y_CENTRE_US + SERVO_SPAN_US};

static_assert(SERVO_X_CAL.minUs >= SERVO_HARD_MIN_US && SERVO_X_CAL.maxUs <= SERVO_HARD_MAX_US,
              "servo X backstop outside SERVO_HARD_MIN_US..SERVO_HARD_MAX_US");
static_assert(SERVO_Y_CAL.minUs >= SERVO_HARD_MIN_US && SERVO_Y_CAL.maxUs <= SERVO_HARD_MAX_US,
              "servo Y backstop outside SERVO_HARD_MIN_US..SERVO_HARD_MAX_US");

// Live trim from the ground station: µs added to a servo's centre, clamped in tvc::setTrim (owner's
// call, 2026-09-14; widened from ±100). ±200 µs is X ±5.2° / Y ±3.9° of nozzle. At full trim the clamp
// swing (X ±330, Y ±440 µs) runs into the ±500 µs backstop on the trimmed side, leaving X 7.8° /
// Y 5.9° of nozzle that way, flagged as saturation, still safe.
constexpr int16_t TVC_TRIM_MAX_US = 200;

// ──────────────────────── Translation estimate ──────────────────────────────
// Low-pass on the range term, which scales the whole velocity estimate:
//   y = α·y + (1 − α)·x
constexpr float RANGE_LPF_ALPHA = 0.9f;  // UNTUNED

// ──────────────────────── Attitude LQI (lib/control/lqr) ─────────────────────
// Solved offline; do not recompute. mass 0.370 kg, arm 0.200 m, I_xx = I_yy = 0.01480 kg·m²
// → B = thrust·arm/I = 49.05. Q = diag(130,130,4,4,10,10), R = 100·I, dt = 0.01.
// Controllability rank 6/6, max closed-loop |pole| 0.9972.
//   x = [r1, r2, dr1, dr2, ir1, ir2]   rad, rad/s, rad·s
//   u = [nozzle X, nozzle Y]           rad
// BOTH B entries are positive on purpose. The reference uses −B_rx/+B_ry, which encodes THEIR
// gimbal handedness as opposite-polarity K rows. Ours keeps K symmetric and puts all mechanical
// sign in ServoCal::dir, which is bench-verified. Do not "fix" this to match theirs.
constexpr uint8_t LQR_STATES = 6;
constexpr uint8_t LQR_INPUTS = 2;
constexpr float   LQR_K[LQR_INPUTS][LQR_STATES] = {
    {1.13552229f, 0.0f, 0.28421317f, 0.0f, 0.29383259f, 0.0f},
    {0.0f, 1.13552229f, 0.0f, 0.28421317f, 0.0f, 0.29383259f},
};

// Global scale on u at boot, the bench sketch's value. Live-adjustable from the ground station
// within 0..LQR_GAIN_SCALE_MAX (owner's call, 2026-09-14).
constexpr float LQR_GAIN_SCALE     = 0.3f;
constexpr float LQR_GAIN_SCALE_MAX = 1.0f;  // 1.0 = the LQI exactly as solved

// Integral clamp, rad·s, from the reference's RX_INTEGRAL_MAX. No leak: an earlier leak treated a
// bench artifact (integrals winding while the loop is open) as a tuning problem.
constexpr float LQR_I_LIMIT = 0.40f;

// Live tuning bounds for the ground station's Tuning panel, as multiples of the solved LQR_K.
// lqr::setGains clamps every received gain to these; the page's sliders are only a convenience.
constexpr float LQR_KTH_RANGE[2] = {0.5f, 1.5f};  // angle term
constexpr float LQR_KQ_RANGE[2]  = {0.5f, 2.0f};  // rate term, more damping is the safer direction
constexpr float LQR_KI_RANGE[2]  = {0.0f, 1.5f};  // integral term, may be switched off entirely

// Integral action is OFF at boot (owner's call, 2026-09-14). Tune angle and rate on the stand,
// read the steady mean nozzle command (the bias), then raise the integral gain live. It must be
// on before free flight: PD alone leaves a steady tilt under any CG or thrust-line offset.
constexpr float LQR_KI_BOOT_FACTOR = 0.0f;  // × the solved integral gain

// Integrators run only in FLYING with the commanded throttle at or above this. Below it the
// vehicle is still on the pad and can't rotate, so the loop is open and they would wind up
// before lift-off; they are held at zero instead. 0.70 (owner's call, 2026-09-15), just under
// the lift-off throttle measured in both 2026-09-15 flights (76-80 %): at the old 0.50 the
// integrators ran for ~2 s on the pad before the vehicle left the ground.
constexpr float LQR_INTEGRATE_MIN_THROTTLE = 0.70f;

// Thrust gain scheduling: nozzle torque ∝ thrust, so u is scaled by NOMINAL / thrust, with a
// floor so the division can't blow up. OFF; do not enable without the user's explicit
// say-so, and never while thrust is zero. There is no throttle → newtons model yet.
constexpr bool  THRUST_SCHED     = false;
constexpr float NOMINAL_THRUST_N = 3.6297f;
constexpr float THRUST_LCLAMP_N  = 0.2f * 9.81f;

// ──────────────────── WiFi SoftAP + WebSocket (lib/wifi_link) ───────────────
// The vehicle is the access point. SSID = prefix + last 4 hex digits of the AP MAC.
// Page at http://192.168.4.1, WebSocket at WIFI_WS_PATH.
constexpr char     WIFI_SSID_PREFIX[] = "mistral-";
constexpr char     WIFI_PASSWORD[]    = "mistral-flight";  // WPA2, 8+ characters
constexpr uint16_t WIFI_HTTP_PORT     = 80;
constexpr char     WIFI_WS_PATH[]     = "/ws";
// Network work runs on core 0; the control loop runs on core 1. Must match
// -D CONFIG_ASYNC_TCP_RUNNING_CORE in platformio.ini.
constexpr uint8_t  WIFI_CORE    = 0;
// Telemetry push rate. 50 Hz (owner's call, 2026-09-14), above the spec's 10–20 Hz so the
// graphs and recordings show servo and rate dynamics; half the 100 Hz loop. Serialising and
// sending run on core 0, so the control loop is unaffected.
constexpr uint16_t TELEMETRY_HZ = 50;
// Link loss: the ground page pings every 250 ms. With no frame from any client for this
// long, loop() disarms, so a dropped phone or a locked screen can't leave motors running.
constexpr uint32_t LINK_TIMEOUT_MS = 1000;

// ──────────────────────── Bench tests (lib/bench) ───────────────────────────
// Bring-up only. While true and DISARMED, loop() steps each nozzle to ±TVC_CLAMP_RAD in turn
// (X+, X−, Y+, Y−) so its real deflection can be measured against the command. The gear
// ratio check. ARM and FLY are unaffected.
constexpr bool     BENCH_SERVO_SWEEP    = false;  // off: servos centred while DISARMED
constexpr uint32_t BENCH_SWEEP_DWELL_MS = 5000;  // hold at each position

}  // namespace cfg
