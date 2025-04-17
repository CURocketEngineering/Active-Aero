#include "core/Setup.h"

void setup() {
    Serial.begin(BAUD_RATE);
    delay(SETUP_DELAY);

    ms24.setup(SERVO_PIN, SERVO_RANGE, SERVO_LOWER_PULSE, SERVO_UPPER_PULSE);
    telemetry.setupSensors();

    // be wary, as we're putting data onto the heap with these new statements. may need to be reconfigured if we have to save space in the future
    dataSaver = new DataSaverBigSD(SD_CHIP_SELECT);
    verticalVelocityEstimator = new VerticalVelocityEstimator();
    ad = new ApogeeDetector(1.0f);
    lp = new LaunchPredictor(ACCEL_THRESHOLD_MS2, LAUNCH_WINDOW_SIZE_MS, LAUNCH_WINDOW_INTERVAL_MS);
    sm = new BurnoutStateMachine(dataSaver, lp, ad, verticalVelocityEstimator);
    ap = new ApogeePredictor(*verticalVelocityEstimator, EMA_ALPHA, MINIMUM_CLIMB_VELOCITY);


    sd_init = dataSaver->begin(); // super super important for comms verification


    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    previousTime = millis();
    targetServoAngle = MIN_DEPLOYMENT_ANGLE;
    startCoastTime = 0;

    #ifdef SIM
        SerialSim::getInstance().begin(&Serial, sm);
    #endif

    delay(SETUP_DELAY); // wait for setup to finish
}
