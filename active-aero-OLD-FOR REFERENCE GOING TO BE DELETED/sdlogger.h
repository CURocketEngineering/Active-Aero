#ifndef SDLOGGER_H
#define SDLOGGER_H

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#include <string>
#include <vector>

#include "telemetry.h"

class SDLogger {
    private:
        File logFile;
    public:
        void setup();
        bool write(std::string log);
        bool writeData(TelemetryData data, std::string flightStatus);
        void close();
};

#endif