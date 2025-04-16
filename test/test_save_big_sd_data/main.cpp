#include <unity.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
#include "data_handling/DataSaverBigSD.h"
#include "data_handling/DataPoint.h"
#include "data_handling/DataNames.h"

DataSaverBigSD dataSaver;

void test_save_data_point() {
    TEST_ASSERT_TRUE(dataSaver.begin());

    DataPoint dp (0, 10); 
    int result = dataSaver.saveDataPoint(dp, 0x01);
    TEST_ASSERT_EQUAL(0, result); // Check if saveDataPoint returns 0 (success)
}

void save_many_data_points() {
    TEST_ASSERT_TRUE(dataSaver.begin());

    for (int i = 0; i < 1000; i++) {
        DataPoint dp (i, i * 10); 
        int result = dataSaver.saveDataPoint(dp, 0x01);
        TEST_ASSERT_EQUAL(0, result); // Check if saveDataPoint returns 0 (success)
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100); // wait for native USB
    }

    UNITY_BEGIN();
    Serial.println("Starting test...");
    TEST_ASSERT_TRUE(dataSaver.begin());
    // call save data point test
    RUN_TEST(test_save_data_point);
    RUN_TEST(save_many_data_points);
    UNITY_END();
}

void loop() {
    // nothing to do here
}