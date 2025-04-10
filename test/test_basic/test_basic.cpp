#include <unity.h>
#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    // while (!Serial) {
    //     delay(100); // wait for native USB
    // }

    // delay(5000);

    // UNITY_BEGIN();
    // Serial.println("Starting test...");
    // // TEST_ASSERT_TRUE(dataSaver.begin());
    // UNITY_END();
}

void loop() {
    // nothing to do here
    Serial.println("Testing");
}