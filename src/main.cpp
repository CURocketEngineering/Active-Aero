#include "config.h"

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

float servoAngle; // servo angle global
double baseAlt;
unsigned long previousTime;
bool sd_init = false;

void communicateVerification(bool sd_init);

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
    ad = new ApogeeDetector(1.0f); // 1.0f is the apogee threshold in meters
    lp = new LaunchPredictor(ACCEL_THRESHOLD_MS2, LAUNCH_WINDOW_SIZE_MS, LAUNCH_WINDOW_INTERVAL_MS); // blanket values ripped from MARTHA

    // NOW initialize objects that use the pointers
    sm = new BurnoutStateMachine(dataSaver, lp, ad, verticalVelocityEstimator);
    ap = new ApogeePredictor(*verticalVelocityEstimator, EMA_ALPHA, MINIMUM_CLIMB_VELOCITY);

    // confirm initialization, setup sd data saver
    sd_init = dataSaver->begin();
    if(dataSaver) { Serial.println("Data saver initialized"); } else { Serial.println("Data saver not initialized"); }
    if(verticalVelocityEstimator) { Serial.println("Vertical velocity estimator initialized"); } else { Serial.println("Vertical velocity estimator not initialized"); }
    if(ad) { Serial.println("Apogee detector initialized"); } else { Serial.println("Apogee detector not initialized"); }
    if(lp) { Serial.println("Launch predictor initialized"); } else { Serial.println("Launch predictor not initialized"); }
    if(sm) { Serial.println("State machine initialized"); } else { Serial.println("State machine not initialized"); }
    if(ap) { Serial.println("Apogee predictor initialized"); } else { Serial.println("Apogee predictor not initialized"); }

    Serial.println(telemetry.getSensorConfig().c_str()); // debugging 

    // LED communication/verif
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println("Finished setup");
    digitalWrite(LED_BUILTIN, HIGH);
    previousTime = millis();

    Serial.println("Entering communication verification...");
    communicateVerification(sd_init);

    #ifdef SIM
    SerialSim::getInstance().begin(&Serial, sm);
    #endif
}


void loop()
{   
    #ifdef SIM
    SerialSim::getInstance().update();
    delay(10);
    #endif

    unsigned long loopStartTime = millis();
    Serial.println("Loop start time: " + String(loopStartTime));

    // init telem & telem sensor recording
    telemData = telemetry.getTelemetry();

    // if you ever change the orientation of the sensors, this WILL probably need to be adjusted
    telemData.sensorData["magnetometer"].magnetic.x = telemData.sensorData["magnetometer"].magnetic.y * -1; 
    telemData.sensorData["magnetometer"].magnetic.y = telemData.sensorData["magnetometer"].magnetic.x;

    double currAlt = telemData.sensorData["altitude"].altitude; // will be used later so store

    // update data points
    aclX.data = telemData.sensorData["acceleration"].acceleration.x;
    aclY.data = telemData.sensorData["acceleration"].acceleration.y;
    aclZ.data = telemData.sensorData["acceleration"].acceleration.z;
    alt.data = currAlt;

    gyroX.data = telemData.sensorData["gyro"].gyro.x;
    gyroX.timestamp_ms = telemData.timestamp;
    gyroY.data = telemData.sensorData["gyro"].gyro.y;
    gyroY.timestamp_ms = telemData.timestamp;
    gyroZ.data = telemData.sensorData["gyro"].gyro.z;
    gyroZ.timestamp_ms = telemData.timestamp;
    temp.data = telemData.sensorData["temperature"].temperature;
    temp.timestamp_ms = telemData.timestamp;
    pres.data = telemData.sensorData["pressure"].pressure;
    pres.timestamp_ms = telemData.timestamp;

    unsigned long currTime = millis();
    aclX.timestamp_ms = aclY.timestamp_ms = aclZ.timestamp_ms = alt.timestamp_ms = currTime; // record timestamp for data points

    // save the data points to their respective data names
    dataSaver->saveDataPoint(aclX, ACCELEROMETER_X);
    dataSaver->saveDataPoint(aclY, ACCELEROMETER_Y);
    dataSaver->saveDataPoint(aclZ, ACCELEROMETER_Z);
    dataSaver->saveDataPoint(alt, ALTITUDE);
    dataSaver->saveDataPoint(temp, TEMPERATURE);
    dataSaver->saveDataPoint(pres, PRESSURE);
    dataSaver->saveDataPoint(gyroX, GYROSCOPE_X);
    dataSaver->saveDataPoint(gyroY, GYROSCOPE_Y);
    dataSaver->saveDataPoint(gyroZ, GYROSCOPE_Z);

    // update state we're in  (Do not update the ap or vve, because the state machine will do that)
    // IMPORTANT: Do not update the vve until after launch, so it's vertical axis determination is correct
    sm->update(aclX, aclY, aclZ, alt);

    // get the current time
    unsigned long nowTime = millis();
    float dt = nowTime - previousTime; // time since last loop
    previousTime = nowTime; 

    if (dt > 0){
        // Save the recriprocal of the time step (ms) to get HZ
        float hz = 1000.0f / dt;
        Serial.println("Hz: " + String(hz));
        // Save this as a data point
        dataSaver->saveDataPoint(DataPoint(nowTime, hz), AVERAGE_CYCLE_RATE);
    }

    Serial.printf("x_hat: \t%f m/s, \t%f m/s/s", verticalVelocityEstimator->getEstimatedVelocity(), verticalVelocityEstimator->getInertialVerticalAcceleration());

    // update apogee predictor
    ap->update(); // update the apogee predictor with the current data points
    float predApogee = ap->getPredictedApogeeAltitude_m();
    dataSaver->saveDataPoint(DataPoint(millis(), predApogee), EST_APOGEE); // save the predicted apogee to the data saver
    // Save time to apogee
    dataSaver->saveDataPoint(DataPoint(millis(), ap->getTimeToApogee_s()), TIME_TO_APOGEE); // save the time to apogee to the data saver
    float targetServoAngle = MIN_DEPLOYMENT_ANGLE; // Default to retracted

    dataSaver->saveDataPoint(DataPoint(millis(), targetServoAngle), FIN_DEPLOYMENT_AMOUNT); // save the servo angle to the data saver

    // we actually want to deploy
    if (sm->getState() == STATE_COAST_ASCENT)
    {
        if(ap->getTimeToApogee_s() >= 0 && ap->getTimeToApogee_s() < FIN_RETRACTION_THRESHOLD_S) // test fin full out to full in time
        {
            Serial.println("Too close to apogee, retracting fins");
            targetServoAngle = MIN_DEPLOYMENT_ANGLE; // re-declare in case SCA -> SD
            ms24.setAngle(targetServoAngle);
        }

        /*** deployment logic here */
        Serial.println("Deploying fins");

        #ifdef TEST_LAUNCH // for the 04/13/2025 flight to just test if the fins will deploy
        targetServoAngle = MAX_DEPLOYMENT_ANGLE; 
        ms24.setAngle(targetServoAngle);
        #endif

        // currently very rudimentary, logic, should be replacing with something a bit more refined
        #ifndef TEST_LAUNCH
        {
            if(ap->getPredictedApogeeAltitude_m() > TARGET_APOGEE + OVERSHOOT_THRESHOLD) // if we're going to overshoot, deploy the fins
            {
                targetServoAngle = MAX_DEPLOYMENT_ANGLE; 
                ms24.setAngle(targetServoAngle);
    
                /**
                 * 
                 * I'm considering making deployment logic a function of the overshoot ->
                 * 
                 * if overshooting, targetServoAngle = amt_overshooting_m * proportional gain 
                 * targetServoAngle = constrain(targetAngle, MIN_DEPLOY, MAX_DEPLOY)
                 * ms24.setAngle(targetServoAngle)
                 * 
                 * gives us a little more control over aggression of deployment as we launch this more & learn in the future, and has a bit
                 * more finesse behind it than the current "if overshooting, max deploy"
                 * 
                 */
            }
            else
            {
                targetServoAngle = MIN_DEPLOYMENT_ANGLE; 
                ms24.setAngle(targetServoAngle);
            }
        }
        #endif
    }

    else    
    { // If we're not in SCA, stay 0 so we don't break the fins
        targetServoAngle = MIN_DEPLOYMENT_ANGLE; // re-declare in case SCA -> SD
        ms24.setAngle(targetServoAngle);
    }

    // comment in/out for servo testing
    // rotate between 0 and 110 degrees 10 times with a 1 second delay
    // for (int i = 0; i < 10; i++)
    // {
    //     Serial.println("Servo test running...");
    //     ms24.setAngle(0);
    //     delay(1000);
    //     ms24.setAngle(110);
    //     delay(1000);
    // }  
}

/***
 * initally deploys fins to show that we are in the communicate verification function
 * retracts fins before entering loop
 * if fins deploy after that point, we have an error
 */
void communicateVerification(bool sd_init)
{
    // moving fins to visually show we're in comms check
    ms24.setAngle(MAX_DEPLOYMENT_ANGLE); // different from full deploy to visually confirm we're undergoing comms verification
    delay(COMMUNICATION_VERIFICATION_DELAY);
    ms24.setAngle(MIN_DEPLOYMENT_ANGLE);
    delay(COMMUNICATION_VERIFICATION_DELAY);

    // init loop to check sensors & sd ptr
    SensorsActivated sensorsActivated = telemetry.getSensorsActivated();
    std::vector<bool> verifiables = {sensorsActivated.mag, sensorsActivated.bmp, sensorsActivated.imu, sd_init}; 
    for (bool verifiable : verifiables)
    {
        if (verifiable)
        {
            ms24.setAngle(MIN_DEPLOYMENT_ANGLE); // in is good if everything is working
            delay(COMMUNICATION_VERIFICATION_DELAY);
        }
        else
        {
            Serial.println("Sensor activation is not working: " + String(verifiable));
            ms24.setAngle(MAX_DEPLOYMENT_ANGLE); // out is bad if something goes wrong
            delay(COMMUNICATION_VERIFICATION_DELAY);
        }
    }
    Serial.println("Finished communication verification, entering main loop...");
    delay(COMMUNICATION_VERIFICATION_DELAY); // delay before returning to main
}
