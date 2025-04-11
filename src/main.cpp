#include <Arduino.h>

#include "telemetry.h"
#include "servointerface.h"
#include "state_estimation\ApogeePredictor.h"
#include "state_estimation\BurnoutStateMachine.h" 
#include "data_handling\DataNames.h"
#include "data_handling\DataPoint.h"
#include "data_handling\DataSaverBigSD.h"

// servo + important board communication vars
#define BAUD_RATE 115200
#define SETUP_DELAY 15000 // millisec
#define SERVO_LOWER_PULSE 500
#define SERVO_UPPER_PULSE 2500
#define SERVO_RANGE 270

// nom nom chips
#define SERVO_PIN 6
#define SD_CHIP_SELECT 5

// math
#define TARGET_APOGEE 10000
#define ACCEL_THRESHOLD_MS2 40 // m/s^2
#define LAUNCH_WINDOW_SIZE_MS 500 // millisec
#define LAUNCH_WINDOW_INTERVAL_MS 25  // millisec
#define OVERSHOOT_THRESHOLD 50.0f // meters
// #define KP_ANGLE 0.05f // aggression of angle change based on predicted overshoot
#define EMA_ALPHA 0.2f
#define MINIMUM_CLIMB_VELOCITY 1.0f

// fin deployment logic variables
#define FIN_RETRACTION_THRESHOLD_S 3.0f
#define MAX_DEPLOYMENT_ANGLE 110.0f
#define MIN_DEPLOYMENT_ANGLE 0.0f

// currently cruft, but this could be very useful if we upgrade our math
#define CROSS_AREA 0.02725801
#define DRAG_COEFFICIENT 0.8
#define ROCKET_MASS 17.23

VerticalVelocityEstimator* verticalVelocityEstimator;
LaunchPredictor *lp;
ApogeeDetector *ad;
ServoInterface ms24;

// these values when initialized are going to be pointing to null val - they're better definined in setup
BurnoutStateMachine* sm = nullptr;
ApogeePredictor* ap = nullptr; // 0.2 is the alpha for the EMA, 1.0 is the minimum climb velocity

DataSaverBigSD* dataSaver;
DataPoint aclX, aclY, aclZ, alt;

Telemetry telemetry; 
TelemetryData telemData; 

float servoAngle; // servo angle global
double baseAlt;
unsigned long previousTime;

void setup()
{
    Serial.begin(BAUD_RATE);
    delay(SETUP_DELAY);
    // put your setup code here, to run once:
    Serial.println("Starting up");
    ms24.setup(SERVO_PIN, SERVO_RANGE, SERVO_LOWER_PULSE, SERVO_UPPER_PULSE);
    telemetry.setupSensors();

    Serial.println("Initializing object pointers (data saver, vve, ad, sm, etc.)...");
    dataSaver = new DataSaverBigSD(SD_CHIP_SELECT); // Ensure SD is initialized correctly

    // init new pointers here (ld, ap, vve, etc.)
    verticalVelocityEstimator = new VerticalVelocityEstimator();
    ad = new ApogeeDetector(OVERSHOOT_THRESHOLD);
    lp = new LaunchPredictor(ACCEL_THRESHOLD_MS2, LAUNCH_WINDOW_SIZE_MS, LAUNCH_WINDOW_INTERVAL_MS); // blanket values ripped from MARTHA

    // NOW initialize objects that use the pointers
    sm = new BurnoutStateMachine(dataSaver, lp, ad, verticalVelocityEstimator);
    ap = new ApogeePredictor(*verticalVelocityEstimator, EMA_ALPHA, MINIMUM_CLIMB_VELOCITY);

    // confirm initialization 
    if(dataSaver) { Serial.println("Data saver initialized"); } else { Serial.println("Data saver not initialized"); }
    if(verticalVelocityEstimator) { Serial.println("Vertical velocity estimator initialized"); } else { Serial.println("Vertical velocity estimator not initialized"); }
    if(ad) { Serial.println("Apogee detector initialized"); } else { Serial.println("Apogee detector not initialized"); }
    if(lp) { Serial.println("Launch predictor initialized"); } else { Serial.println("Launch predictor not initialized"); }
    if(sm) { Serial.println("State machine initialized"); } else { Serial.println("State machine not initialized"); }
    if(ap) { Serial.println("Apogee predictor initialized"); } else { Serial.println("Apogee predictor not initialized"); }

    Serial.println(telemetry.getSensorConfig().c_str()); // debugging and shizzle

    // LED communication/verif
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println("Finished setup");
    digitalWrite(LED_BUILTIN, HIGH);
    previousTime = millis();

    Serial.println("Entering communication verification...");
    // communicateVerification();
}


void loop()
{
    unsigned long loopStartTime = millis();
    Serial.println("Loop start time: " + String(loopStartTime));

    // init telem & telem sensor recording
    telemData = telemetry.getTelemetry();

    // if you ever change the orientation of the sensors, this WILL probably need to be adjusted
    telemData.sensorData["magnetometer"].magnetic.x = telemData.sensorData["magnetometer"].magnetic.y * -1; 
    telemData.sensorData["magnetometer"].magnetic.y = telemData.sensorData["magnetometer"].magnetic.x;

    double currAlt = telemData.sensorData["altitude"].altitude; // will be used later so store
    Serial.println("Retrieved data altimeter and magnetometer data");

    // update data points
    aclX.data = telemData.sensorData["acceleration"].acceleration.x;
    aclY.data = telemData.sensorData["acceleration"].acceleration.y;
    aclZ.data = telemData.sensorData["acceleration"].acceleration.z;
    alt.data = currAlt;
    Serial.println("Retrieved accelerometer data");

    unsigned long currTime = millis();
    aclX.timestamp_ms = aclY.timestamp_ms = aclZ.timestamp_ms = alt.timestamp_ms = currTime; // record timestamp for data points

    Serial.println("Saving data...");
    // save the data points to their respective data names
    dataSaver->saveDataPoint(aclX, ACCELEROMETER_X);
    dataSaver->saveDataPoint(aclY, ACCELEROMETER_Y);
    dataSaver->saveDataPoint(aclZ, ACCELEROMETER_Z);
    dataSaver->saveDataPoint(alt, ALTITUDE);

    // update state we're in, update vve, update altitude prediction
    verticalVelocityEstimator->update(aclX, aclY, aclZ, alt);
    ap->update();
    sm->update(aclX, aclY, aclZ, alt);
    Serial.println("Updated state machine, current state: " + String(sm->getState()));

    float vAccel = verticalVelocityEstimator->getInertialVerticalAcceleration(); // board has to be facing upwards

    // complicated time calculations (contact Samuel Pupke or Mikey Schoonmaker if you have any questions/need a walk through)
    unsigned long nowTime = millis();
    float dt = previousTime - nowTime;
    previousTime = nowTime; // hold onto your bootstraps kid, this is where it gets WILD

    Serial.println("Time equations passed, thank goodness our processor made it");
    Serial.printf("x_hat: \t%f m/s, \t%f m/s/s", verticalVelocityEstimator->getEstimatedVelocity(), verticalVelocityEstimator->getInertialVerticalAcceleration());

    double predApogee = ap->getPredictedApogeeAltitude_m();
    Serial.println("Apogee Prediction: " + String(predApogee) + "m");
    Serial.println("Getting flight status");

    float targetServoAngle = MIN_DEPLOYMENT_ANGLE; // Default to retracted

    // we actually want to deploy
    if (sm->getState() == STATE_COAST_ASCENT)
    {
        if(ap->getTimeToApogee_s() >= 0 && ap->getTimeToApogee_s() < FIN_RETRACTION_THRESHOLD_S) // test fin full out to full in time
        {
            Serial.println("Too close to apogee, retracting fins");
            targetServoAngle = MIN_DEPLOYMENT_ANGLE; // re-declare in case SCA -> SD
            ms24.setPercentAngle(targetServoAngle);
        }

        Serial.println("Deploy them thangs");
        /*** deployment logic here */

    }

    else    
    { // If we're not in SCA, stay 0 so we don't break the fins
        Serial.println("Not in coast ascent, retracting fins");
        targetServoAngle = MIN_DEPLOYMENT_ANGLE; // re-declare in case SCA -> SD
        ms24.setAngle(targetServoAngle);
    }

    // // comment in/out for servo testing
    // // rotate between 0 and 110 degrees 10 times with a 1 second delay
    // for (int i = 0; i < 10; i++)
    // {
    //     Serial.println("Servo test running...");
    //     ms24.setAngle(0);
    //     delay(1000);
    //     ms24.setAngle(110);
    //     delay(1000);
    // }  
}


void communicateVerification(/* consider adding a var to be passed from pointer setup - if they don't, we don't verify */)
{
    // ms24.setPercentAngle(50);
    // delay(2000);
    SensorsActivated sensorsActivated = telemetry.getSensorsActivated();
    //   bool sdActivated = sdLogger.isInitialized();
    // check the big sd
    // std::vector<bool> verifiables = {sensorsActivated.mag, sensorsActivated.bmp, sensorsActivated.imu, sdActivated}; // big sd
    bool flag = false;
    // for (bool verifiable : verifiables)
    // {
    //     if (verifiable)
    //     {
    //     ms24.setPercentAngle(0); // in is good if everything is working
    //     delay(2000);
    //     }
    //     else
    //     {
    //     ms24.setPercentAngle(100); // out is bad if something goes wrong
    //     delay(2000);
    //     flag = true;
    //     }
    //     ms24.setPercentAngle(50);
    //     delay(2000);
    // }
    // ms24.setPercentAngle(100 * flag);
    // delay(2000);
}



//      "All of the calculations show it can't work. There's only one thing to do: make it work."