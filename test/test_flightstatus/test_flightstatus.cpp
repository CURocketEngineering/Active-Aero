#include "unity.h"

#include <Arduino.h>

// #include "telemetry.h"
#include <flightstatus.h>

// void setUp(void) {
//     // set stuff up here
// }

// void tearDown(void) {
//     // clean stuff up here
// }

void test_initial_stage_is_armed(void) {
    FlightStatus status;
    TEST_ASSERT_EQUAL(ARMED, status.getStage());
}

void test_stage_transitions(void) {
    FlightStatus status;
    status.newTelemetry(10, 0); // Ascent
    TEST_ASSERT_EQUAL(ASCENT, status.getStage());

    status.newTelemetry(-10, 0); // Coast
    TEST_ASSERT_EQUAL(COAST, status.getStage());

    status.newTelemetry(0, 0); // Apogee
    TEST_ASSERT_EQUAL(APOGEE, status.getStage());

    status.newTelemetry(0, 0); // Descent
    TEST_ASSERT_EQUAL(DESCENT, status.getStage());

    status.newTelemetry(0, 10); // On ground
    TEST_ASSERT_EQUAL(ONGROUND, status.getStage());
}

void test_apogee_detection(void) {
    FlightStatus status;
    // Fill altitudeDeque with values that should trigger apogee detection
    for (int i = 0; i < 48; i++) {
        status.newTelemetry(0, i);
    }
    TEST_ASSERT_EQUAL(APOGEE, status.getStage());
}

void test_ground_detection(void) {
    FlightStatus status;
    // Fill altitudeDeque with values that should trigger ground detection
    for (int i = 0; i < 16; i++) {
        status.newTelemetry(0, 100);
    }
    status.newTelemetry(0, 19.9);
    TEST_ASSERT_EQUAL(DESCENT, status.getStage());
    
    status.newTelemetry(0, 20);
    TEST_ASSERT_EQUAL(ONGROUND, status.getStage());
}

int runUnityTests(void) {
  UNITY_BEGIN();
  RUN_TEST(test_initial_stage_is_armed);
  RUN_TEST(test_stage_transitions);
  RUN_TEST(test_apogee_detection);
  RUN_TEST(test_ground_detection);
  return UNITY_END();
}

void setup()
{
    delay(2000);
    
    runUnityTests();
}

void loop() {}