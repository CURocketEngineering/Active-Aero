#include <gtest/gtest.h>
#include "flightstatus.h"

TEST(FlightStatusTest, InitialStageIsArmed) {
    FlightStatus status;
    ASSERT_EQ(status.getStage(), ARMED);
}

TEST(FlightStatusTest, StageTransitions) {
    FlightStatus status;
    status.newTelemetry(10, 0); // Ascent
    ASSERT_EQ(status.getStage(), ASCENT);

    status.newTelemetry(-10, 0); // Coast
    ASSERT_EQ(status.getStage(), COAST);

    status.newTelemetry(0, 0); // Apogee
    ASSERT_EQ(status.getStage(), APOGEE);

    status.newTelemetry(0, 0); // Descent
    ASSERT_EQ(status.getStage(), DESCENT);

    status.newTelemetry(0, 10); // On ground
    ASSERT_EQ(status.getStage(), ONGROUND);
}

TEST(FlightStatusTest, ApogeeDetection) {
    FlightStatus status;
    // Fill altitudeDeque with values that should trigger apogee detection
    for (int i = 0; i < 48; i++) {
        status.newTelemetry(0, i);
    }
    ASSERT_EQ(status.getStage(), APOGEE);
}

TEST(FlightStatusTest, GroundDetection) {
    FlightStatus status;
    // Fill altitudeDeque with values that should trigger ground detection
    for (int i = 0; i < 16; i++) {
        status.newTelemetry(0, 100);
    }
    status.newTelemetry(0, 19.9);
    ASSERT_EQ(status.getStage(), DESCENT);

    status.newTelemetry(0, 20);
    ASSERT_EQ(status.getStage(), ONGROUND);
}