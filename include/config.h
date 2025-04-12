#ifndef CONFIG_H
#define CONFIG_H

#include "ArduinoHAL.h"

#include "telemetry.h"
#include "servointerface.h"
#include "state_estimation/ApogeePredictor.h"
#include "state_estimation/BurnoutStateMachine.h" 
#include "data_handling/DataNames.h"
#include "data_handling/DataPoint.h"
#include "data_handling/DataSaverBigSD.h"

#define LAUNCH_TYPE 1 // 1 for simple launch, 0 advanced launch

// mission configuration variables
#define TARGET_APOGEE 10000
#define BAUD_RATE 115200

// delays embedded within functions
#define SETUP_DELAY 15000 // millisec
#define COMMUNICATION_VERIFICATION_DELAY 2000 // millisec

// chip setup
#define SERVO_PIN 6
#define SD_CHIP_SELECT 5

// servo + important board communication vars
#define SERVO_LOWER_PULSE 500
#define SERVO_UPPER_PULSE 2500
#define SERVO_RANGE 270

// fin deployment logic variables
#define FIN_RETRACTION_THRESHOLD_S 1.5f
#define MAX_DEPLOYMENT_ANGLE 110.0f
#define HALFWAY_DEPLOYED 55.0f
#define MIN_DEPLOYMENT_ANGLE 0.0f

// anything related to the math behind deployment
#define OVERSHOOT_THRESHOLD 50.0f // meters
// #define KP_ANGLE 0.05f // aggression of angle change based on predicted overshoot, potentially work into logic
#define EMA_ALPHA 0.2f
#define MINIMUM_CLIMB_VELOCITY 1.0f

// launch predictor constants
#define ACCEL_THRESHOLD_MS2 40 // m/s^2
#define LAUNCH_WINDOW_SIZE_MS 500 // millisec
#define LAUNCH_WINDOW_INTERVAL_MS 25  // millisec

// currently cruft, but this could be very useful if we upgrade our math
#define CROSS_AREA 0.02725801
#define DRAG_COEFFICIENT 0.8
#define ROCKET_MASS 17.23

#endif 