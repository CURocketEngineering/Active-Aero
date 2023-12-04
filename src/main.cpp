#include <Arduino.h>

#include "telemetry.h"
#include "apogeeprediction.h"
#include "flightstatus.h"
#include "sdlogger.h"
#include "ahrs.h"

SDLogger sdLogger;
Telemetry telemetry;
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
  Serial.println("Getting Telemetry");
  TelemetryData telemData = telemetry.getTelemetry();
  Serial.println("Got Telemetry");
  Serial.println(telemData.sensorData["acceleration"].acceleration.x);
  Serial.println(telemData.sensorData["acceleration"].acceleration.y);
  Serial.println(telemData.sensorData["altitude"].altitude);

  Serial.println("Updating AHRS State");
  
  
  ahrs.update(telemData.sensorData["gyro"].gyro.x, 
              telemData.sensorData["gyro"].gyro.y, 
              telemData.sensorData["gyro"].gyro.z, 
              telemData.sensorData["acceleration"].acceleration.x, 
              telemData.sensorData["acceleration"].acceleration.y, 
              telemData.sensorData["acceleration"].acceleration.z, 
              telemData.sensorData["magnetometer"].magnetic.x, 
              telemData.sensorData["magnetometer"].magnetic.y, 
              telemData.sensorData["magnetometer"].magnetic.z);
  Serial.println("AHRS State Updated");
  
  float rx, ry, rz;
  ahrs.getRotationVector(&rx,&ry,&rz);
  Serial.printf("rx=%f \try=%f \trz=%f", rx, ry, rz);
  float gx, gy, gz;
  ahrs.getGravityVector(&gx, &gy, &gz);
  Serial.printf("\tgx=%f \tgy=%f \tgz=%f", gx, gy, gz);
  float ax, ay, az;
  Serial.printf("\tax=%f \tay=%f \taz=%f\n", ax, ay, az);
  /*Serial.printf("mx=%f \tmy=%f \tmz=%f\n",telemData.sensorData["magnetometer"].magnetic.x, 
              telemData.sensorData["magnetometer"].magnetic.y, 
              telemData.sensorData["magnetometer"].magnetic.z);*/ //having some issues where the magnetometer isnt always connected
  sleep(0.1);
}