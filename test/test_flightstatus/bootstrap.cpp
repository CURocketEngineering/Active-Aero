#include "unity.h"

#if defined(ARDUINO)
#include <Arduino.h>

#include "telemetry.h"
#include "flightstatus.h"
#include "test_flightstatus.cpp"

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

#else
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    // if you plan to use GMock, replace the line above with
    // ::testing::InitGoogleMock(&argc, argv);

    if (RUN_ALL_TESTS())
    ;

    // Always return zero-code and allow PlatformIO to parse results
    return 0;
}
#endif