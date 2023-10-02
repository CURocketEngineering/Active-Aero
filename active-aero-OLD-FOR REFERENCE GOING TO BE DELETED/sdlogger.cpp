#include "sdlogger.h"

void SDLogger::setup() {
  Serial.print("Initializing SD card...");

  if(!SD.begin(10)) {
    Serial.println("Initialization failed!");
  } else {
    Serial.println("initialization done.");
  }

  logFile = SD.open("LOGS.TXT", FILE_WRITE, true);

  if(logFile) {
    Serial.print("Opened Log");
  }
}

bool SDLogger::write(std::string log) {
    if (logFile) {
        logFile.println(log.c_str());
        return true;
    } else {
        return false;
    }
}

bool SDLogger::writeData(TelemetryData data, std::string flightStatus) {
    std::string dataString = "";
    return write(dataString);
}

void SDLogger::close() {
    logFile.close();
}