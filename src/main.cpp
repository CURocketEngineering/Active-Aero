#include <Arduino.h>

#include "data_handling/DataNames.h"
#include "telemetry.h"
#include "servointerface.h"
#include "state_estimation\ApogeePredictor.h"
#include "state_estimation\BurnoutStateMachine.h" 
#include "data_handling\DataPoint.h"

#define TARGET_APOGEE 10000
#define SERVO_PIN 6
#define SD_CHIP_SELECT 5
#define CROSS_AREA = 0.02725801
#define DRAG_COEFFICIENT = 0.8
#define ROCKET_MASS = 17.23

IDataSaver* dataSaver;
VerticalVelocityEstimator* verticalVelocityEstimator;
LaunchPredictor *lp;
ApogeeDetector *ad;
ServoInterface ms24;

BurnoutStateMachine sm(dataSaver, lp, ad, verticalVelocityEstimator);
DataPoint aclX, aclY, aclZ, alt;
ApogeePredictor ap(*verticalVelocityEstimator, 0.2, 1.0); // 0.2 is the alpha for the EMA, 1.0 is the minimum climb velocity

Telemetry telemetry; // do we care about this
TelemetryData telemData; // still i ask, do we care

const float finRetractionThreshold = 3; // seconds
float servoAngle; // servo angle global
double baseAlt;
unsigned long previousTime;

void setup()
{
    Serial.begin(115200);
    delay(15000);
    // put your setup code here, to run once:
    Serial.println("Starting up");
    Serial.println("hello world");
    ms24.setup(SERVO_PIN, 270, 500, 2500);
    ms24.setAngle(90);
    delay(500);
    ms24.setAngle(10);
    delay(500);
    ms24.setAngle(90);
    // SETUP BIG SD LOGGER HERE
    telemetry.setupSensors();

    Serial.println(telemetry.getSensorConfig().c_str()); // debugging and shizzle

    pinMode(LED_BUILTIN, OUTPUT);

    Serial.println("Finished setup");
    digitalWrite(LED_BUILTIN, HIGH);
    previousTime = millis();

    // communicateVerification();
}


void loop()
{
    // init telem & telem sensor recording
    telemData = telemetry.getTelemetry();
    telemData.sensorData["magnetometer"].magnetic.x = telemData.sensorData["magnetometer"].magnetic.y * -1;
    telemData.sensorData["magnetometer"].magnetic.y = telemData.sensorData["magnetometer"].magnetic.x;

    // accel data storage
    double accel[] = {
        telemData.sensorData["acceleration"].acceleration.x,
        telemData.sensorData["acceleration"].acceleration.y,
        telemData.sensorData["acceleration"].acceleration.z
    };

    // send data to state machine
    aclX.data = accel[0];
    aclY.data = accel[1];
    aclZ.data = accel[2];
    alt.data = telemData.sensorData["altitude"].altitude;

    aclX.timestamp_ms = aclY.timestamp_ms = aclZ.timestamp_ms = alt.timestamp_ms = millis(); // record timestamp for data points

    dataSaver->saveDataPoint(aclX, ACCELEROMETER_X);
    dataSaver->saveDataPoint(aclY, ACCELEROMETER_Y);
    dataSaver->saveDataPoint(aclZ, ACCELEROMETER_Z);
    dataSaver->saveDataPoint(alt, ALTITUDE);

    // update state we're in, update vve, update altitude prediction
    verticalVelocityEstimator->update(aclX, aclY, aclZ, alt);
    ap.update();
    sm.update(aclX, aclY, aclZ, alt);

    float vAccel = verticalVelocityEstimator->getInertialVerticalAcceleration(); // board has to be facing upwards

    // complicated time calculations
    unsigned long nowTime = millis();
    float dt = previousTime - nowTime;
    previousTime = nowTime;

    Serial.printf("x_hat: \t%f m/s, \t%f m/s/s", verticalVelocityEstimator->getEstimatedVelocity(), verticalVelocityEstimator->getInertialVerticalAcceleration());

    double predApogee = ap.getPredictedApogeeAltitude_m();
    Serial.println("Apogee Prediction: " + String(predApogee) + "m");
    Serial.println("Getting flight status");

    // if(ap.getTimeToApogee_s() < finRetractionThreshold) // test fin full out to full in time
    // {
    //     ms24.setPercentAngle(0);
    // }

    // else
    // {
    //     // Deployment logic for fins - stay 0 so we don't break them
    //     if (sm.getState() == STATE_ARMED || sm.getState() == STATE_POWERED_ASCENT || sm.getState() == STATE_DESCENT)
    //     {
    //         servoAngle = 0;
    //         ms24.setAngle(servoAngle);
    //     }
    //     // we actually want to deploy
    //     else if (sm.getState() == STATE_COAST_ASCENT)
    //     {
    //         if(predApogee > TARGET_APOGEE)
    //         {
    //             servoAngle = 110; /* create function to deploy at an angle based on drag coefficient, for now use 110 */
    //             ms24.setAngle(servoAngle);
    //         }
    //         else
    //         {
    //             servoAngle = 0;
    //             ms24.setAngle(servoAngle);
    //         }
    //     }
    // }

    // ms24.setAngle(110);
    // delay(1000);
    // ms24.setAngle(0);
    // delay(1000);   
}


void communicateVerification()
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