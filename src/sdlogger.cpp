#include "sdlogger.h"

SDLogger::SDLogger() {
    logFile = "/logs.txt";
    telemFile = "/telemetry.csv";
}

SDLogger::SDLogger(std::string logFP = "/logs.txt", std::string telemFP = "/telemetry.csv") {
    logFile = logFP.c_str();
    telemFile = telemFP.c_str();
}

void SDLogger::setup() {
  Serial.print("Initializing SD card...");

  pinMode(10, OUTPUT);
  if(!SD.begin(10)) {
    Serial.println("Initialization failed!");
  } else {
    Serial.println("Initialization done.");
  }

  Serial.println("Finished SD Setup!");
}

bool SDLogger::writeLog(std::string log) {
    const char* c_log = (log + '\n').c_str();
    appendFile(SD, logFile, c_log);
    return true;
}

bool SDLogger::writeData(TelemetryData data) {
    std::string dataString = "";
    return write(dataString);
}

void SDLogger::readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void SDLogger::writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

bool SDLogger::appendFile(fs::FS &fs, const char * path, const char * message){
    // Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        // Serial.println("Failed to open file for appending");
        return false;
    }
    if(file.print(message)){
        // Serial.println("Message appended");
        return true;
    } else {
        // Serial.println("Append failed");
        return false;
    }
    file.close();
}

void SDLogger::renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void SDLogger::deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}


// void createDir(fs::FS &fs, const char * path){
//     Serial.printf("Creating Dir: %s\n", path);
//     if(fs.mkdir(path)){
//         Serial.println("Dir created");
//     } else {
//         Serial.println("mkdir failed");
//     }
// }

// void removeDir(fs::FS &fs, const char * path){
//     Serial.printf("Removing Dir: %s\n", path);
//     if(fs.rmdir(path)){
//         Serial.println("Dir removed");
//     } else {
//         Serial.println("rmdir failed");
//     }
// }
