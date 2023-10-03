#include <Arduino.h>

#include "telemetry.h"
#include "apogeeprediction.h"
#include "flightstatus.h"
#include "sdlogger.h"

Servo aaservo;
bool servoExtended = false;
double timeServoExtended = 0;

Telemetry telem;
ApogeePrediction apogeePrediction(17.23, 0.8, 0.02725801, 1371.6);
FlightStatus flightStatus;
SDLogger sdLogger;

double lastTime = 0;
double baseAlt;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  while (!Serial) {}
  Serial.println("Starting up");

  aaservo.attach(1);
  aaservo.write(5);
  Serial.println("Bound Servo");

  telem.setupTelem();

  sdLogger.setup();

  delay(5000);
  std::map<std::string,std::vector<double>> telemData = telem.getTelemetry();
  baseAlt = telemData["altitude"][0];

  pulseServo();
}

void loop() {
  if(sdLogger.write("Hello World")) {
    Serial.println("Wrote SD");
  } else {
    Serial.println("Failed to write SD");
  }
  
  telem.pollBNO();
  std::map<std::string,std::vector<double>> telemData = telem.getTelemetry();

  double* accel = &telemData["acceleration"][0];
  double* orien = &telemData["orientation"][0];
  double press = telemData["pressure"][0];
  double temp = telemData["temperature"][0];
  double alt = telemData["altitude"][0] - baseAlt;

  double predApogee = apogeePrediction.predictApogee(accel, orien, press, temp, alt);

  if(millis() > timeServoExtended + 250 && servoExtended) {
    pulseServo();
  }

  if(predApogee > 1150 || alt > 1270 ) { // && accel[0] < 1.5 && accel[1] < 1.5 && accel[2] < 1.5) {
    if(!servoExtended) {
      pulseServo();
    }
  }

  

  if(millis() > lastTime + 250) {
    lastTime = millis();
  std::vector<double> acceleration = telemData["acceleration"];
  std::vector<double> gyroscope = telemData["gyroscope"];
  std::vector<double> magnetometer = telemData["magnetometer"];
  std::vector<double> orientation = telemData["orientation"];
  std::vector<double> pressure = telemData["pressure"];
  std::vector<double> temperature = telemData["temperature"];
  std::vector<double> altitude = telemData["altitude"];

  Serial.print("PREDICTED APOGEE: ");
  Serial.print(predApogee);
  Serial.println();

  Serial.print("SERVO EXTENDED: ");
  Serial.println(servoExtended);

  Serial.print("Acceleration: ");
  Serial.print(acceleration[0]);
  Serial.print(", ");
  Serial.print(acceleration[1]);
  Serial.print(", ");
  Serial.println(acceleration[2]);

  Serial.print("Gyroscope: ");
  Serial.print(gyroscope[0]);
  Serial.print(", ");
  Serial.print(gyroscope[1]);
  Serial.print(", ");
  Serial.println(gyroscope[2]);

  Serial.print("Magnetometer: ");
  Serial.print(magnetometer[0]);
  Serial.print(", ");
  Serial.print(magnetometer[1]);
  Serial.print(", ");
  Serial.println(magnetometer[2]);

  Serial.print("Orientation: ");
  Serial.print(orientation[0]);
  Serial.print(", ");
  Serial.print(orientation[1]);
  Serial.print(", ");
  Serial.println(orientation[2]);

  Serial.print("Pressure: ");
  Serial.println(pressure[0]);

  Serial.print("Temperature: ");
  Serial.println(temperature[0]);

  Serial.print("Altitude: ");
  Serial.println(alt);
  }
}

void pulseServo() {
  if(servoExtended) {
    timeServoExtended = millis();
    aaservo.write(0);
    servoExtended = false;
  } else {
    aaservo.write(65);
    servoExtended = true;
  }
}
