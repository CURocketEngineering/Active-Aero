/**
 * Fin‑deployment integration tests (no mocks)
 *
 * Hardware wiring for the pulse‑width probe
 * ──────────────────────────────────────────
 *   SERVO_PIN  ──►  Servo signal line
 *                  (also jumper to INT_PIN)
 *
 *   INT_PIN    ──►  Reads the 50 Hz PWM signal so the test
 *                  can measure the pulse‑width the ServoInterface
 *                  is actually generating.
 *
 *   GND is common between the servo and the MCU.
 */

#include <unity.h>
#include "config.h"

// ----------------- hardware test constants -----------------
#ifndef INT_PIN
#define INT_PIN 2                 // free digital pin that can read PWM
#endif

static ServoInterface ms24;       // the same driver class used in main.cpp
static DataSaverBigSD dataSaver(SD_CHIP_SELECT);

static VerticalVelocityEstimator vve;
static ApogeeDetector            ad(OVERSHOOT_THRESHOLD);
static LaunchDetector           lp(ACCEL_THRESHOLD_MS2,
                                    LAUNCH_WINDOW_SIZE_MS,
                                    LAUNCH_WINDOW_INTERVAL_MS);
static BurnoutStateMachine      *sm  = nullptr;
static ApogeePredictor          *ap  = nullptr;

static Telemetry  telemetry;
static bool       sensors_ready = false;

// ---------------------------------------------------------------------------
// Utility: convert the live PWM pulse width we measure on INT_PIN back to
// a servo angle so we can assert against it.
// ---------------------------------------------------------------------------
static int pulseWidthToAngle(uint32_t us)
{
    // constrain() prevents negative overflow if the servo rail glitches
    us = constrain(us, SERVO_LOWER_PULSE, SERVO_UPPER_PULSE);
    return map(us, SERVO_LOWER_PULSE, SERVO_UPPER_PULSE, 0, SERVO_RANGE);
}

// ---------------------------------------------------------------------------
// TEST 1 – Servo retracts
// ---------------------------------------------------------------------------
void test_servo_retracts_to_min()
{
    ms24.setAngle(MIN_DEPLOYMENT_ANGLE);
    delay(500);                                           // give the servo time
    uint32_t pw = pulseIn(INT_PIN, HIGH, 30000);          // µs
    int angle   = pulseWidthToAngle(pw);

    TEST_ASSERT_UINT32_WITHIN(150, SERVO_LOWER_PULSE, pw);  // 150 µs tolerance
    TEST_ASSERT_INT_WITHIN(5, MIN_DEPLOYMENT_ANGLE, angle); // ±5 °
}

// ---------------------------------------------------------------------------
// TEST 2 – Servo deploys
// ---------------------------------------------------------------------------
void test_servo_deploys_to_max()
{
    ms24.setAngle(MAX_DEPLOYMENT_ANGLE);
    delay(500);
    uint32_t pw = pulseIn(INT_PIN, HIGH, 30000);
    int angle   = pulseWidthToAngle(pw);

    TEST_ASSERT_UINT32_WITHIN(150, SERVO_UPPER_PULSE, pw);
    TEST_ASSERT_INT_WITHIN(5, MAX_DEPLOYMENT_ANGLE, angle);
}

// ---------------------------------------------------------------------------
// Helper: run the real flight code long enough to reach Coast‑Ascent
//         (about 2–3 s on the bench).  We purposely do not fake any
//         sensor readings; instead we just wait for the IMU to report
//         near‑zero acceleration, which causes LaunchDetector to time‑out
//         and the state machine to enter SCA.
// ---------------------------------------------------------------------------
static void armStateMachine()
{
    unsigned long t0 = millis();
    while (sm->getState() != STATE_COAST_ASCENT && millis() - t0 < 5000)
    {
        TelemetryData td = telemetry.getTelemetry();
        DataPoint ax (td.sensorData["acceleration"].acceleration.x, millis());
        DataPoint ay (td.sensorData["acceleration"].acceleration.y, millis());
        DataPoint az (td.sensorData["acceleration"].acceleration.z, millis());
        DataPoint alt(td.sensorData["altitude"].altitude,             millis());

        sm->update(ax, ay, az, alt);
        delay(10);
    }
}

// ---------------------------------------------------------------------------
// TEST 3 – Complete fin‑deployment logic in SCA
// ---------------------------------------------------------------------------
void test_state_machine_enters_SCA_and_drives_fins()
{
    // Make sure we really got into SCA
    armStateMachine();
    TEST_ASSERT_EQUAL(STATE_COAST_ASCENT, sm->getState());

    // Scenario A – Overshoot predicted ⇒ fins should deploy
    while (ap->getPredictedApogeeAltitude_m() <= TARGET_APOGEE + OVERSHOOT_THRESHOLD)
        ; // wait until real VVE predicts overshoot (you can tilt the altimeter)

    // One more state‑machine update to execute the branch
    TelemetryData td = telemetry.getTelemetry();
    DataPoint ax (td.sensorData["acceleration"].acceleration.x, millis());
    DataPoint ay (td.sensorData["acceleration"].acceleration.y, millis());
    DataPoint az (td.sensorData["acceleration"].acceleration.z, millis());
    DataPoint alt(td.sensorData["altitude"].altitude,             millis());
    sm->update(ax, ay, az, alt);

    delay(300);
    uint32_t pw_deploy = pulseIn(INT_PIN, HIGH, 30000);
    int angle_deploy   = pulseWidthToAngle(pw_deploy);
    TEST_ASSERT_INT_WITHIN(5, MAX_DEPLOYMENT_ANGLE, angle_deploy);

    // Scenario B – Not overshooting ⇒ fins retract
    while (ap->getPredictedApogeeAltitude_m() > TARGET_APOGEE)
        ; // gently move the altimeter lower or wait for estimate to settle

    sm->update(ax, ay, az, alt);
    delay(300);
    uint32_t pw_retract = pulseIn(INT_PIN, HIGH, 30000);
    int angle_retract   = pulseWidthToAngle(pw_retract);
    TEST_ASSERT_INT_WITHIN(5, MIN_DEPLOYMENT_ANGLE, angle_retract);
}

// ---------------------------------------------------------------------------
// TEST 4 – SD card still works
// ---------------------------------------------------------------------------
void test_big_sd_can_save()
{
    TEST_ASSERT_TRUE(dataSaver.begin());

    DataPoint dp (0, 10);
    int rc = dataSaver.saveDataPoint(dp, ACCELEROMETER_X);
    TEST_ASSERT_EQUAL(0, rc);
}

// ---------------------------------------------------------------------------
// Unity SETUP / LOOP
// ---------------------------------------------------------------------------
void setUp(void)  {}   // unused but required by Unity
void tearDown(void) {} // unused but required by Unity

void setup()
{
    Serial.begin(115200);
    while (!Serial) {}

    // ── hardware init ────────────────────────────────────────────────────
    pinMode(INT_PIN, INPUT);
    ms24.setup(SERVO_PIN, SERVO_RANGE, SERVO_LOWER_PULSE, SERVO_UPPER_PULSE);

    telemetry.setupSensors();
    sensors_ready = telemetry.getSensorsActivated().imu &&
                    telemetry.getSensorsActivated().bmp &&
                    telemetry.getSensorsActivated().mag;

    sm = new BurnoutStateMachine(&dataSaver, &lp, &ad, &vve);
    ap = new ApogeePredictor(vve, EMA_ALPHA, MINIMUM_CLIMB_VELOCITY);

    // ── run the tests ────────────────────────────────────────────────────
    UNITY_BEGIN();

    RUN_TEST(test_servo_retracts_to_min);
    RUN_TEST(test_servo_deploys_to_max);

    if (sensors_ready)
        RUN_TEST(test_state_machine_enters_SCA_and_drives_fins);
    else
        Serial.println("Skipping SCA test: sensors not ready");

    RUN_TEST(test_big_sd_can_save);

    UNITY_END();
}

void loop() { /* nothing */ }
