#include "common/config.h"
#include "common/globals.h"


/***
 * initally deploys fins to show that we are in the communicate verification function
 * retracts fins before entering loop
 * if fins deploy after that point, we have an error
 */
void commsVerification(bool sd_init) {
    ms24.setAngle(MAX_DEPLOYMENT_ANGLE);
    delay(COMMUNICATION_VERIFICATION_DELAY);
    ms24.setAngle(MIN_DEPLOYMENT_ANGLE);
    delay(COMMUNICATION_VERIFICATION_DELAY);

    SensorsActivated sensorsActivated = telemetry.getSensorsActivated();
    std::vector<bool> verifiables = {sensorsActivated.mag, sensorsActivated.bmp, sensorsActivated.imu, sd_init};
    for (bool verifiable : verifiables) {
        ms24.setAngle(verifiable ? MIN_DEPLOYMENT_ANGLE : MAX_DEPLOYMENT_ANGLE);
        delay(COMMUNICATION_VERIFICATION_DELAY);
    }
    delay(COMMUNICATION_VERIFICATION_DELAY);
}
