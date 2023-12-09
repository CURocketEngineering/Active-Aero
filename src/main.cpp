#include <Arduino.h>

#include "telemetry.h"
#include "apogeeprediction.h"
#include "flightstatus.h"
#include "sdlogger.h"
#include "ahrs.h"

SDLogger sdLogger;
Telemetry telemetry;
FlightStatus flightStatus;
AHRS ahrs;

double baseAlt;

void setup() {
  sleep(5);
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Starting up");

  sdLogger.setup();

  telemetry.setupSensors();
  sdLogger.writeLog("Setup complete");
  Serial.println(telemetry.getSensorConfig().c_str());
  sdLogger.writeLog(telemetry.getSensorConfig());

  ahrs.begin(115200);//sample frequency 
  ahrs.setRotationVector(0,0,0);
  
  Serial.println("Finished setup");
}

void loop() {
  TelemetryData telemData = telemetry.getTelemetry();
  
  ahrs.update(telemData.sensorData["gyro"].gyro.x, 
              telemData.sensorData["gyro"].gyro.y, 
              telemData.sensorData["gyro"].gyro.z, 
              telemData.sensorData["acceleration"].acceleration.x, 
              telemData.sensorData["acceleration"].acceleration.y, 
              telemData.sensorData["acceleration"].acceleration.z, 
              telemData.sensorData["magnetometer"].magnetic.x, 
              telemData.sensorData["magnetometer"].magnetic.y, 
              telemData.sensorData["magnetometer"].magnetic.z);

  float rx, ry, rz;
  ahrs.getRotationVector(&rx,&ry,&rz);
  // Serial.printf("rx=%f \try=%f \trz=%f", rx, ry, rz);
  float gx, gy, gz;
  ahrs.getGravityVector(&gx, &gy, &gz);
  // Serial.printf("\tgx=%f \tgy=%f \tgz=%f", gx, gy, gz);

  AHRSMap ahrsData;
  ahrsData["rx"] = rx;
  ahrsData["ry"] = ry;
  ahrsData["rz"] = rz;
  ahrsData["gx"] = gx;
  ahrsData["gy"] = gy;
  ahrsData["gz"] = gz;

  flightStatus.newTelemetry(telemData.sensorData["acceleration"].acceleration.z, telemData.sensorData["pressure"].pressure);

  sdLogger.writeData(telemData, ahrsData, flightStatus.getStage());
}