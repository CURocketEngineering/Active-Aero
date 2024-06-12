#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>

#include <map>
#include <string>
#include <vector>
#include <sstream>

#define SEALEVELPRESSURE_HPA (1013.25)

// struct BmpData {
//     float temperature;
//     float pressure;
//     float altitude;
// };

// struct SensorData : public sensors_event_t {
//     union {
//         sensors_vec_t acceleration; /**< acceleration values are in meter per second
//                                      per second (m/s^2) */
//         sensors_vec_t magnetic;     /**< magnetic vector values are in micro-Tesla (uT) */
//         sensors_vec_t orientation;  /**< orientation values are in degrees */
//         sensors_vec_t gyro;         /**< gyroscope values are in rad/s */
//         BmpData barometric;
//     };
// };

typedef sensors_event_t SensorData;

typedef std::map<std::string, SensorData> SensorDataMap;

struct TelemetryData {
    uint64_t timestamp;
    SensorDataMap sensorData;
};

struct SensorsActivated {
    bool mag = false;
    bool bmp = false;
    bool imu = false;
};

class Telemetry {
   private:
    SensorsActivated sensorsActivated;
    double baseAltitude;

    Adafruit_LSM6DSOX imu;
    Adafruit_LIS3MDL mag;
    Adafruit_BMP3XX bmp;


   public:
    Telemetry();

    void setupSensors();

    bool setupMag();
    bool setupImu();
    bool setupBmp();

    std::string getSensorConfig();

    TelemetryData getTelemetry();
};

#endif