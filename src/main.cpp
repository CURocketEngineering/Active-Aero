#include <Arduino.h>

#include "telemetry.h"
#include "apogeeprediction.h"
#include "flightstatus.h"
#include "sdlogger.h"
#include "ahrs.h"
#include "kf-2d.h"
#include "servointerface.h"

void communicateVerification(); // Function prototype

SDLogger sdLogger;
Telemetry telemetry;
FlightStatus flightStatus(16);
AHRS ahrs;
KF2D KF;
ServoInterface ms24;

float rocketMass = 17.23; // need to calculate wet and dry mass of rocket, make two vars
float dragCoefficent = 0.8; // need to calculate drag coefficient of rocket without flaps
float crossArea = 0.02725801; // need to calculate crossarea of rocket without flaps, as well as a new var with flaps extended
float targetApogee = 1000; // This does nothing for now, you can ignore it
float servoAngle; // servo angle global

// ApogeePrediction apogeePrediction(rocketMass, dragCoefficent, crossArea, targetApogee);

double baseAlt;
KF2D::MeasurementVector measurement;
TelemetryData telemData;

unsigned long previousTime;

void setup()
{
  delay(15000);
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting up");
  ms24.setup(38, 270, 500, 2500);
  ms24.setPercentAngle(50);
  delay(1000);

  sdLogger.setup();

  telemetry.setupSensors();
  
  communicateVerification();
  
  
  sdLogger.writeLog("Setup complete");
  Serial.println(telemetry.getSensorConfig().c_str());
   sdLogger.writeLog(telemetry.getSensorConfig());

  ahrs.begin(); // param: sampling frequency
  // the next 2 lines should be run when launch state is detected instead of at start

  measurement = {telemData.sensorData["altitude"].altitude, (-1)*telemData.sensorData["acceleration"].acceleration.z}; //want y then ay
  KF.InitializeKalmanFilter(measurement);

  pinMode(LED_BUILTIN, OUTPUT);


  Serial.println("Finished setup");
  digitalWrite(LED_BUILTIN, HIGH);
  previousTime = millis();
}

void loop()
{

  telemData = telemetry.getTelemetry();
  //Serial.println("Got telemetry data");
  /*
  Serial.println("Telemetry data:");
  for (auto const& sensor : telemData.sensorData)
  {
    Serial.print(sensor.first.c_str());
    Serial.print(": ");
    Serial.print(sensor.second.acceleration.x);
    Serial.print(", ");
    Serial.print(sensor.second.acceleration.y);
    Serial.print(", ");
    Serial.print(sensor.second.acceleration.z);
    Serial.println();
  }
  Serial.println();
  */


  // Fix orientation of LIS3MDL to LSM6DSOX
  telemData.sensorData["magnetometer"].magnetic.x = telemData.sensorData["magnetometer"].magnetic.y * -1;
  telemData.sensorData["magnetometer"].magnetic.y = telemData.sensorData["magnetometer"].magnetic.x;

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
   Serial.printf("rx=%f \try=%f \trz=%f", rx, ry, rz); //still very rough, try to avoid using if at all possible
  float gx, gy, gz;
   ahrs.getGravityVector(&gx, &gy, &gz);
   Serial.printf("\tgx=%f \tgy=%f \tgz=%f", gx, gy, gz);

  AHRSMap ahrsData;
  ahrsData["rx"] = rx;
  ahrsData["ry"] = ry;
  ahrsData["rz"] = rz;
  ahrsData["gx"] = gx;
  ahrsData["gy"] = gy;
  ahrsData["gz"] = gz;

  //Calculate the orientation using the gravity vector
  double euler[3];
  euler[0] = atan2(gy, gz);
  euler[1] = atan2(-gx, sqrt(gy * gy + gz * gz));
  euler[2] = 0;

  // Vert Velocity Prediction
  double accel[] = {telemData.sensorData["acceleration"].acceleration.x,
                    telemData.sensorData["acceleration"].acceleration.y,
                    telemData.sensorData["acceleration"].acceleration.z};
  //double euler[] = {ahrsData["rx"], ahrsData["ry"], ahrsData["rz"]};
  double vAccel = ApogeePrediction::getVertAccel(accel, euler);
  // double vAccel = telemData.sensorData["acceleration"].acceleration.z-9.81; // This "vAccel" value is only when the board is facing upwards, not at any other orientation

  // The next 3 lines should be run on loop after launch is detected
  // // measurement = {telemData.sensorData["altitude"].altitude, (float)(-9.81+telemData.sensorData["acceleration"].acceleration.z)}; //want y then ay- NOT g //eventually needs to move to AHRS vertical accel
  measurement = {telemData.sensorData["altitude"].altitude, (float)(vAccel)};
  Serial.printf("altitude = %f, \t y-acceleration = %f\t", measurement[0], measurement[1]);
  unsigned long nowTime = millis();
  float dt = previousTime - nowTime;
  previousTime = nowTime;
  // // float dt = 0.25;
  KF.Update(measurement);
  KF.Predict();
  Serial.printf("x_hat: \t%f m, \t%f m/s, \t%f m/s/s", KF.x_hat[0], KF.x_hat[1], KF.x_hat[2]);

  KFData kfData = {
      acceleration: KF.x_hat[2],
      velocity: KF.x_hat[1],
      drift: KF.x_hat[0]
  };

  // testflight app and link on slack

  // Apogee Prediction
  // REPLACE APPRED W SENSOR AY
  // double predApogee = apogeePrediction.predictApogee(KF.x_hat[1], telemData.sensorData["pressure"].pressure, telemData.sensorData["temperature"].temperature, telemData.sensorData["altitude"].altitude);
  double predApogee = ApogeePrediction::newPredictApogee(KF.x_hat[1], telemData.sensorData["altitude"].altitude, telemData.sensorData["pressure"].pressure, telemData.sensorData["temperature"].temperature, dragCoefficent, rocketMass, crossArea);
  Serial.println("Apogee Prediction: " + String(predApogee) + "m");
  Serial.println("Getting flight status");
  flightStatus.newTelemetry(telemData.sensorData["acceleration"].acceleration.z, telemData.sensorData["altitude"].altitude);
  Serial.printf("Flight Status: %s\n", flightStatus.getStageString().c_str());
  
  if (flightStatus.getStage() == ARMED
      || flightStatus.getStage() == ASCENT
      || flightStatus.getStage() == APOGEE
      || flightStatus.getStage() == DESCENT
      || flightStatus.getStage() == ONGROUND)
  {
    servoAngle = 0;
    ms24.setAngle(servoAngle);
  }
  else if (flightStatus.getStage() == COAST)
  {
    servoAngle = 180;
    ms24.setAngle(servoAngle);
  }

  //Serial.printf("servo-angle = %f \t", servoAngle);


  // Serial.println("VAccel: " + String(vAccel));
  sdLogger.writeData(telemData, kfData, vAccel, predApogee, flightStatus.getStageString(), servoAngle);

  
}

void communicateVerification()
{
  ms24.setPercentAngle(50);
  delay(2000);
  SensorsActivated sensorsActivated = telemetry.getSensorsActivated();
  bool sdActivated = sdLogger.isInitialized();
  std::vector<bool> verifiables = {sensorsActivated.mag, sensorsActivated.bmp, sensorsActivated.imu, sdActivated};
  bool flag = false;
  for (bool verifiable : verifiables)
  {
    if (verifiable)
    {
      ms24.setPercentAngle(0); // in is good if everything is working
      delay(2000);
    }
    else
    {
      ms24.setPercentAngle(100); // out is bad if something goes wrong
      delay(2000);
      flag = true;
    }
    ms24.setPercentAngle(50);
    delay(2000);
  }
  ms24.setPercentAngle(100 * flag);
  delay(2000);
}