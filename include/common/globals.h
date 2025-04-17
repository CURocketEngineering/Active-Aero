#include "telemetry.h"
#include "servointerface.h"
#include "state_estimation/ApogeePredictor.h"
#include "state_estimation/BurnoutStateMachine.h" 
#include "data_handling/DataNames.h"
#include "data_handling/DataPoint.h"
#include "data_handling/DataSaverBigSD.h"


VerticalVelocityEstimator* verticalVelocityEstimator;
LaunchPredictor *lp;
ApogeeDetector *ad;
ServoInterface ms24;

// these values when initialized are going to be pointing to null val - they're better definined in setup
BurnoutStateMachine* sm = nullptr;
ApogeePredictor* ap = nullptr; // 0.2 is the alpha for the EMA, 1.0 is the minimum climb velocity

DataSaverBigSD* dataSaver;
DataPoint aclX, aclY, aclZ, alt, temp, pres, gyroX, gyroY, gyroZ;

Telemetry telemetry; 
TelemetryData telemData; 

extern float servoAngle;
extern float targetServoAngle;
extern double baseAlt;
extern unsigned long previousTime;
extern bool sd_init;
extern uint32_t startCoastTime;
extern ServoInterface ms24;