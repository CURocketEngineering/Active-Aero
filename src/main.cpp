#include <Arduino.h>

#include "telemetry.h"
#include "apogeeprediction.h"
#include "flightstatus.h"
#include "sdlogger.h"
#include "ahrs.h"
#include "kf-2d.h"

SDLogger sdLogger;
Telemetry telemetry;
FlightStatus flightStatus;
AHRS ahrs;
KF2D KF;

double baseAlt;
KF2D::MeasurementVector measurement;

void setup()
{
  sleep(5);
  // put your setup code here, to run once:
  Serial.begin(115200);
  // while (!Serial) {}
  Serial.println("Starting up");

  //sdLogger.setup();

  telemetry.setupSensors();
  //sdLogger.writeLog("Setup complete");
  Serial.println(telemetry.getSensorConfig().c_str());
  //sdLogger.writeLog(telemetry.getSensorConfig());

  ahrs.begin(115200); // sample frequency
  ahrs.setRotationVector(0, 0, 0);
  KF.InitializeKalmanFilter();

  Serial.println("Finished setup");
}

void loop()
{
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
  ahrs.getRotationVector(&rx, &ry, &rz);
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
  
  measurement = {telemData.sensorData["altitude"].altitude, telemData.sensorData["acceleration"].acceleration.z}; //want y then ay
  Serial.printf("altitude = %f, \t acceleration = %f\n", measurement[0], measurement[1]);
  KF.Update(measurement);
  KF.Predict();
  //Serial.printf("x_hat: \t%fm, \t%fm/s, \t%fm/s/s\n", KF.x_hat[0], KF.x_hat[1], KF.x_hat[2]);
  //sdLogger.writeData(telemData, ahrsData, flightStatus.getStage());
}