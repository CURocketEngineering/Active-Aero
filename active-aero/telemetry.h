#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <string>
#include <vector>
#include <map>

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include "Adafruit_BMP3XX.h"

typedef std::map<std::string,std::vector<double>> TelemetryData;

class Telemetry {
  private:
    double baseAltitude;

    std::vector<double> acceleration;

    sh2_SensorValue_t lAcceleration;
    sh2_SensorValue_t lOrientation;
    sh2_SensorValue_t lMagnetometer;
    sh2_SensorValue_t lGyroscope;

    
    Adafruit_BNO08x bno08x;
    sh2_SensorValue_t sensorValue;

    Adafruit_BMP3XX bmp;
  public:
    Telemetry();
    void setupTelem();

    void setReports();
    void pollBNO();
    std::map<std::string,std::vector<double>> getTelemetry();
};

#endif