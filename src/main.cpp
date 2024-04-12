#include <Arduino.h>

#include "telemetry.h"
//#include "apogeeprediction.h"
#include "flightstatus.h"
#include "sdlogger.h"
//#include "ahrs.h"
//#include "kf-2d.h"

SDLogger sdLogger;  
Telemetry telemetry;
FlightStatus flightStatus;
//AHRS ahrs;
//KF2D KF;

//double baseAlt;
//KF2D::MeasurementVector measurement;
TelemetryData telemData;

int i;

void setup()
{
  delay(5000);
  i=0;
  // put your setup code here, to run once:
  Serial.begin(115200);
  // while (!Serial) {}
  Serial.println("Starting up");

  sdLogger.setup();

  telemetry.setupSensors();
  sdLogger.writeLog("Setup complete");
  Serial.println(telemetry.getSensorConfig().c_str());
  sdLogger.writeLog(telemetry.getSensorConfig());

  //ahrs.begin(115200); // sample frequency
  //ahrs.setRotationVector(0, 0, 0);
  //the next 2 lines should be run when launch state is detected instead of at start

  // HAD TO COMMMENT OUT CAUSE SPACE CONSTRAINTS
  // measurement = {telemData.sensorData["altitude"].altitude, (-1)*telemData.sensorData["acceleration"].acceleration.z}; //want y then ay
  // KF.InitializeKalmanFilter(measurement);

  Serial.println("Finished setup");
}

void loop()
{
  i++;
  if(i<10000) {
  Serial.println("Getting telemetry");}
  telemData = telemetry.getTelemetry();
  if(i<10000) {
  Serial.println("Got telemetry");}
  /*
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

  // AHRSMap ahrsData;
  // ahrsData["rx"] = rx;
  // ahrsData["ry"] = ry;
  // ahrsData["rz"] = rz;
  // ahrsData["gx"] = gx;
  // ahrsData["gy"] = gy;
  // ahrsData["gz"] = gz;

  
  // Serial.printf("altitude = %f, \t y-acceleration = %f\t", measurement[0], measurement[1]);
  // //The next 3 lines should be run on loop after launch is detected
  // measurement = {telemData.sensorData["altitude"].altitude, (float)(-9.81+telemData.sensorData["acceleration"].acceleration.z)}; //want y then ay- NOT g //eventually needs to move to AHRS vertical accel
  // KF.Update(measurement);
  // KF.Predict();
  // Serial.printf("x_hat: \t%f m, \t%f m/s, \t%f m/s/s\n", KF.x_hat[0], KF.x_hat[1], KF.x_hat[2]);
  
  */
  //sdLogger.writeData(telemData, ahrsData, flightStatus.getStage());
  //Serial.println("Getting flight status");
  //flightStatus.newTelemetry(telemData.sensorData["acceleration"].acceleration.z, telemData.sensorData["pressure"].pressure);
  if(i<10000){
  Serial.println("Writing to SD");  }
  sdLogger.writeData(telemData);
  if(i<10000){
  Serial.printf("Looping %d", i);}
  if(i=10000){
  sdLogger.readFile(SD, "/telemetry.csv");}
}