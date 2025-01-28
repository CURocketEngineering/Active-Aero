#include <Arduino.h>

#include "flightstatus.h"
#include "servointerface.h"
#include "sdlogger.h"

ServoInterface ms24;
SDLogger sdLogger;
FlightStatus flightStatus(16);

void loop() {
  flightStatus.flightStage = ARMED;
  if (flightStatus.getStage() == ARMED) {
    Serial.print("ARMED");
    ms24.setPercentAngle(0);
  }
  sleep(10);

  flightStatus.flightStage = COAST;
  if (flightStatus.getStage() == COAST) {
    Serial.print("COAST");
    ms24.setPercentAngle(50);
  }
  sleep(20);


  flightStatus.flightStage = APOGEE;
  if (flightStatus.getStage() == APOGEE) {
    Serial.print("APOGEE");
    ms24.setPercentAngle(0);
  }
  sleep(10);
}

void setup() {
  delay(15000);
  Serial.begin(115200);
  Serial.println("Starting up");
  ms24.setup(38, 270, 500, 2500);
  ms24.setPercentAngle(50);
  delay(1000);
  sdLogger.setup();
}