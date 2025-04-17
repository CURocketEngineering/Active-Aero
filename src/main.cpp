#include "common/config.h"
#include "common/globals.h"
#include "simulation/Serial_Sim.h"

#include "core/Setup.h"
#include "core/CommunicationVerification.h"
#include "core/TelemetryProcessing.h"

void main()
{   
    setup();
    commsVerification(sd_init); // check sensors & sd card

    while(1)
    {

        #ifdef SIM
            SerialSim::getInstance().update();
            delay(10);
        #endif

        unsigned long loopStartTime = millis();

        updateTelem(); // update the telemetry data

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
            // Save this as a data point
            dataSaver->saveDataPoint(DataPoint(nowTime, hz), AVERAGE_CYCLE_RATE);
        }

        // update apogee predictor
        ap->update(); // update the apogee predictor with the current data points
        float predApogee = ap->getPredictedApogeeAltitude_m();
        dataSaver->saveDataPoint(DataPoint(millis(), predApogee), EST_APOGEE); // save the predicted apogee to the data saver
        // Save time to apogee
        dataSaver->saveDataPoint(DataPoint(millis(), ap->getTimeToApogee_s()), TIME_TO_APOGEE); // save the time to apogee to the data saver


        if (sm->getState() == STATE_COAST_ASCENT) // we actually want to deploy
        {
            if(startCoastTime == 0)
            {
                startCoastTime = millis();
            }

            if(ap->getTimeToApogee_s() < FIN_RETRACTION_THRESHOLD_S || (millis() - startCoastTime) < FIN_EJECTION_DELAY_MS) // test fin full out to full in time
            {
                targetServoAngle = MIN_DEPLOYMENT_ANGLE; // re-declare in case SCA -> SD
            }
            
            #ifdef TEST_FIN_DEPLOYMENT // for the 04/13/2025 flight to just test if the fins will deploy
            else // if we're in SCA, deploy to the maximum possible angle (-10/+10)
            {
                targetServoAngle = MAX_DEPLOYMENT_ANGLE; 
            }
            #endif


            // currently very rudimentary, logic, should be replacing with something a bit more refined
            #ifndef TEST_FIN_DEPLOYMENT
            else {
                if(ap->getPredictedApogeeAltitude_m() > TARGET_APOGEE + OVERSHOOT_THRESHOLD) // if we're going to overshoot, deploy the fins. + threshold so we don't make a sinusoid nightmare
                {
                    targetServoAngle = MAX_DEPLOYMENT_ANGLE; 
        
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
                }
            }
            #endif


        }
        else    
        { // If we're not in SCA, stay 0 so we don't break the fins
            targetServoAngle = MIN_DEPLOYMENT_ANGLE; 
        }

        // set angle and log at the end of each iteration
        dataSaver->saveDataPoint(DataPoint(millis(), targetServoAngle), FIN_DEPLOYMENT_AMOUNT); // save the servo angle to the data saver
        ms24.setAngle(targetServoAngle); 
    }
}
