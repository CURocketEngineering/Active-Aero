#include <Arduino.h>

#include "telemetry.h"
// #include "apogeeprediction.h"
// #include "flightstatus.h"
#include "sdlogger.h"

SDLogger sdLogger;
Telemetry telemetry;

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
  
  Serial.println("Finished setup");
}

void loop() {
  Serial.println("Getting Telemetry");
  TelemetryData telemData = telemetry.getTelemetry();
  Serial.println("Got Telemetry");
  Serial.println(telemData.sensorData["acceleration"].acceleration.x);
  Serial.println(telemData.sensorData["acceleration"].acceleration.y);
  Serial.println(telemData.sensorData["altitude"].altitude);
  sleep(1);
}