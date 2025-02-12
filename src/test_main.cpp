// #include <Arduino.h>
// #include <ESP32Servo.h>

// #include "flightstatus.h"
// //#include "servointerface.h"
// //#include "sdlogger.h"
// Servo myServo;

// //ServoInterface ms24;
// //SDLogger sdLogger;
// FlightStatus flightStatus(16);

// void loop() {
//   flightStatus.flightStage = ARMED;
//   if (flightStatus.getStage() == ARMED) {
//     Serial.print("ARMED");
//     //ms24.setAngle(0);
//     myServo.write(30);
//     //ms24.servo.writeMicroseconds(700);
//   }
//   sleep(5);

//   flightStatus.flightStage = COAST;
//   if (flightStatus.getStage() == COAST) {
//     Serial.print("COAST");
//     //ms24.setAngle(180);
//     myServo.write(-180);
//     //ms24.servo.writeMicroseconds(2300);
//   }
//   sleep(10);


//   flightStatus.flightStage = APOGEE;
//   if (flightStatus.getStage() == APOGEE) {
//     Serial.print("APOGEE");
//     //ms24.setAngle(0);
//     myServo.write(30);
//     //ms24.servo.writeMicroseconds(700);
//   }
//   sleep(5);
// }

// void setup() {
//   delay(15000);
//   Serial.begin(115200);
//   Serial.println("Starting up");
//   //ms24.setup(38, 180, 500, 2500);
//   //ms24.setPercentAngle(50);
//   myServo.attach(38, 500, 2500);
//   delay(1000);
//   //sdLogger.setup();
// }