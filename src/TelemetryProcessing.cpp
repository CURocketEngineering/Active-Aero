#include "core/TelemetryProcessing.h"

void updateTelem()
{
    #ifdef SIM
        SerialSim::getInstance().update();
        delay(10);
    #endif

    // init telem & telem sensor recording
    telemData = telemetry.getTelemetry();

    // if you ever change the orientation of the sensors, this WILL probably need to be adjusted
    telemData.sensorData["magnetometer"].magnetic.x = telemData.sensorData["magnetometer"].magnetic.y * -1; 
    telemData.sensorData["magnetometer"].magnetic.y = telemData.sensorData["magnetometer"].magnetic.x;

    currAlt = telemData.sensorData["altitude"].altitude; // will be used later so store

    // update data points
    aclX.data = telemData.sensorData["acceleration"].acceleration.x;
    aclY.data = telemData.sensorData["acceleration"].acceleration.y;
    aclZ.data = telemData.sensorData["acceleration"].acceleration.z;
    alt.data = currAlt;

    gyroX.data = telemData.sensorData["gyro"].gyro.x;
    gyroY.data = telemData.sensorData["gyro"].gyro.y;
    gyroZ.data = telemData.sensorData["gyro"].gyro.z;
    temp.data = telemData.sensorData["temperature"].temperature;
    pres.data = telemData.sensorData["pressure"].pressure;

    // give everything the same timestamp for eventual byte5 integration
    unsigned long currTime = millis();
    aclX.timestamp_ms = aclY.timestamp_ms = aclZ.timestamp_ms = alt.timestamp_ms = currTime; 
    gyroX.timestamp_ms = gyroY.timestamp_ms = gyroZ.timestamp_ms = currTime; 
    temp.timestamp_ms = currTime; 
    pres.timestamp_ms = currTime; 

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
}