// main.cpp — Mistral thrust-vectored coaxial drone
//
// Wiring only. setup() brings modules up in a safe order; loop() runs at
// cfg::CONTROL_LOOP_HZ on core 1: IMU → attitude → state machine → LQI → servos, with
// ground-station commands applied at the loop's own rate. Sensors other than the IMU,
// translation and the display are still stubs.
//
// Every module header is included so `pio run` compiles every library — the
// dependency finder only builds libraries reachable from src/.

#include <Arduino.h>

#include <mistral_config.h>

#include <attitude.h>
#include <bench.h>
#include <bmp388.h>
#include <display.h>
#include <imu.h>
#include <lqr.h>
#include <mtf01.h>
#include <safety.h>
#include <state_machine.h>
#include <throttle.h>
#include <translation.h>
#include <tvc.h>
#include <wifi_link.h>

// A failed LEDC attach produces no pulses at all while everything else looks alive.
// Stop here instead of running.
static void halt(const char* why) {
    for (;;) {
        Serial.println(why);
        delay(1000);
    }
}

// micros() of the previous control tick. Set at the end of setup(), so the first dt is one tick.
static uint32_t lastTickUs = 0;

void setup() {
    Serial.begin(cfg::SERIAL_BAUD);

    if (!throttle::init()) halt("HALT: throttle::init - LEDC attach failed");  // FIRST: both ESCs at min
    if (!tvc::init()) halt("HALT: tvc::init - LEDC attach failed");            // servos parked at centre
    state_machine::init();

    display::init();

    // Keep the vehicle upright and still until "IMU ok": init() measures gyro bias.
    if (imu::init()) {
        const imu::Vec3 b = imu::gyroBias_dps();
        Serial.printf("IMU ok  gyro bias %+.3f %+.3f %+.3f dps  UI filter %s\n", b.x, b.y, b.z,
                      imu::uiFilterSet() ? "set" : "NOT SET - chip reset filter");
        attitude::init(imu::accelMean_g());
    } else {
        Serial.printf("IMU init FAILED: begin() = %d (-3: wrong bus or address)\n", imu::beginCode());
    }
    bmp388::init();
    mtf01::init();

    if (!wifi_link::init()) halt("HALT: wifi_link::init - SoftAP failed");
    lastTickUs = micros();
}

void loop() {
    const uint32_t nowUs = micros();
    if (nowUs - lastTickUs < 1000000 / cfg::CONTROL_LOOP_HZ) return;
    const float dt = (nowUs - lastTickUs) * 1e-6f;
    lastTickUs     = nowUs;
    const uint32_t now = millis();

    const imu::Reading       imuReading = imu::read();
    // The gyro offset tracker learns only on the ground — never while FLYING.
    const attitude::Estimate att        = attitude::update(
        imuReading, dt, state_machine::state() != state_machine::State::FLYING);

    // Ground-station commands: recorded by the network side, applied here at the loop's
    // own rate. The throttle level goes first, so an arm or fly is judged against the zero
    // the network side set when it received it. Kill is applied last so it always wins.
    const wifi_link::Commands cmd = wifi_link::takeCommands();
    throttle::setNormalised(cmd.throttle);
    throttle::setBalance(cmd.balance);  // clamped to cfg::ESC_BALANCE_RANGE
    if (cmd.arm) {
        const bool ok = state_machine::requestArm({imuReading.valid, att.valid, cmd.throttle});
        wifi_link::ack(cmd.armId, ok, "attitude not ready");
    }
    if (cmd.fly) {
        const bool ok = state_machine::requestFly();
        wifi_link::ack(cmd.flyId, ok, "not ARMED, or throttle not zero");
    }
    if (cmd.disarm) {
        state_machine::disarm();
        wifi_link::ack(cmd.disarmId, true);
    }
    tvc::setTrim(tvc::Axis::X, cmd.trimXUs);
    tvc::setTrim(tvc::Axis::Y, cmd.trimYUs);
    static uint32_t gainsSeq = 0;
    if (cmd.gainsSeq != gainsSeq) {
        gainsSeq           = cmd.gainsSeq;
        const lqr::Gains g = lqr::setGains(cmd.gains);  // clamped to the config ranges
        Serial.printf("gains scale %.2f  X %.3f %.3f %.3f  Y %.3f %.3f %.3f\n", g.scale, g.x.kth,
                      g.x.kq, g.x.ki, g.y.kth, g.y.kq, g.y.ki);
    }
    if (!cmd.linkAlive && state_machine::state() != state_machine::State::DISARMED) {
        Serial.println("Link lost - disarmed");
        state_machine::disarm();
    }
    if (cmd.kill) {
        state_machine::disarm();
        wifi_link::ack(cmd.killId, true);
    }
    if (cmd.zero) {
        // Re-zero the IMU on the ground. imu::calibrate() blocks ~1.5 s, so DISARMED only —
        // nothing is being controlled — and it keeps the old zero if the vehicle moves.
        const bool disarmed = state_machine::state() == state_machine::State::DISARMED;
        const bool ok       = disarmed && imu::calibrate();
        if (ok) {
            attitude::init(imu::accelMean_g());
            const imu::Vec3 b = imu::gyroBias_dps();
            Serial.printf("IMU re-zeroed  gyro bias %+.3f %+.3f %+.3f dps\n", b.x, b.y, b.z);
        }
        wifi_link::ack(cmd.zeroId, ok,
                       !disarmed                ? "DISARM first"
                       : imu::beginCode() != 1  ? "IMU not running"
                                                : "vehicle moved - hold it still and retry");
        lastTickUs = micros();  // the calibration blocked the loop: keep that gap out of the next dt
    }

    throttle::update();
    state_machine::update();

    // DISARMED: controller bypassed, integrals zero, servos centred. ARMED: PD on the servos.
    // FLYING: full LQI. No throttle → thrust model yet, so scheduling is handed nominal thrust.
    lqr::Output ctl{};
    if (state_machine::controlActive()) {
        ctl = lqr::update(att, state_machine::integrating(),
                          safety::saturation(tvc::status(), throttle::status()),
                          cfg::NOMINAL_THRUST_N, dt);
        tvc::setDeflection(ctl.u1, ctl.u2);
    } else {
        lqr::reset();
        if (cfg::BENCH_SERVO_SWEEP) bench::servoSweep(now);
        else tvc::centre();
    }

    static uint32_t lastPrintMs = 0;
    if (now - lastPrintMs >= cfg::SERIAL_PRINT_MS) {
        lastPrintMs = now;
        constexpr float   R2D = 57.29578f;
        const tvc::Status t   = tvc::status();
        Serial.printf("%-8s r %+6.2f %+6.2f  dr %+6.1f %+6.1f  i %+6.3f %+6.3f  u %+5.2f %+5.2f  us %4u %4u%s\n",
                      state_machine::name(state_machine::state()), att.r1 * R2D, att.r2 * R2D,
                      att.dr1 * R2D, att.dr2 * R2D, ctl.ir1, ctl.ir2, ctl.u1 * R2D, ctl.u2 * R2D,
                      t.x_us, t.y_us,
                      !imuReading.valid ? "  IMU INVALID" : (att.valid ? "" : "  converging"));
    }

    wifi_link::publish({now, throttle::status(), tvc::status(), att, state_machine::state(), ctl,
                        state_machine::integrating(), lqr::gains()});
}
