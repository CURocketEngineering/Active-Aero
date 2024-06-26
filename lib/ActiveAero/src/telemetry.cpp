#include "telemetry.h"

Telemetry::Telemetry() {}

void Telemetry::setupSensors()
{
    // Serial.println("Initializing BNO and BMP");
    // while (!bmp.begin_I2C()) {}
    // Serial.println("BMP initialized");
    // while (!bno08x.begin_I2C(0x4A)) {}
    // Serial.println("BNO initialized");
    // Serial.println("BNO and BMP initialized");

    if (!sensorsActivated.imu)
    {
        sensorsActivated.imu = setupImu();
    }
    if (!sensorsActivated.mag)
    {
        sensorsActivated.mag = setupMag();
    }
    if (!sensorsActivated.bmp)
    {
        sensorsActivated.bmp = setupBmp();
    }
}

bool Telemetry::setupImu()
{
    if (!imu.begin_I2C())
    {
        Serial.println("Failed to find LSM6DSOX chip");
        return false;
    }

    Serial.println("LSM6DSOX Found!");

    imu.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);

    return true;
}

bool Telemetry::setupMag()
{
    if (!mag.begin_I2C(0x1C))
    {
        if (!mag.begin_I2C(0x1E)) // alternate address 0x1E from 0x1C that happens sometimes
        {
            Serial.println("Failed to find LIS3MDL chip");
            return false;
        }
    }

    Serial.println("LIS3MDL Found!");

    mag.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    mag.setDataRate(LIS3MDL_DATARATE_155_HZ);
    mag.setRange(LIS3MDL_RANGE_4_GAUSS);

    mag.setIntThreshold(500);
    mag.configInterrupt(false, false, true, // enable z axis
                        true,               // polarity
                        false,              // don't latch
                        true);              // enabled!

    return true;
}

bool Telemetry::setupBmp()
{
    if (!bmp.begin_I2C())
    {
        Serial.println("Failed to find BMP390 chip");
        return false;
    }
    Serial.println("BMP390 Found!");
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_100_HZ);
    return true;
}

TelemetryData Telemetry::getTelemetry()
{
    SensorDataMap data;

    SensorData acceleration;
    SensorData gyro;
    SensorData temp;
    imu.getEvent(&acceleration, &gyro, &temp);
    data["acceleration"] = acceleration;
    data["gyro"] = gyro;

    SensorData magnetometerData;
    mag.getEvent(&magnetometerData);

    data["magnetometer"] = magnetometerData;

    if (bmp.performReading())
    {
        SensorData temperatureData;
        temperatureData.temperature = bmp.temperature;
        data["temperature"] = temperatureData;

        SensorData pressureData;
        pressureData.pressure = bmp.pressure;
        data["pressure"] = pressureData;

        SensorData altitudeData;
        altitudeData.altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
        data["altitude"] = altitudeData;
    }

    TelemetryData telemData;
    telemData.timestamp = millis();
    telemData.sensorData = data;

    return telemData;
}

std::string Telemetry::getSensorConfig()
{
    std::stringstream config;

    config << "Activated:\nImu: " << sensorsActivated.imu
           << "\nMag: " << sensorsActivated.mag
           << "\nBMP: " << sensorsActivated.bmp << "\n";

    if (sensorsActivated.imu)
    {
        config << "\nIMU Config:\nAccel Range: " << imu.getAccelRange()
               << "\n - Gyro Range: " << imu.getGyroRange()
               << "\n - Accel Data Rate: " << imu.getAccelDataRate()
               << "\n - Gyro Data Rate: " << imu.getGyroDataRate() << "\n";
    }

    if (sensorsActivated.mag)
    {
        config << "\nMag Config:\nPerformance Mode: " << mag.getPerformanceMode()
               << "\n - Operation Mode: " << mag.getOperationMode()
               << "\n - Data Rate: " << mag.getDataRate()
               << "\n - Range: " << mag.getRange()
               << "\n - Interrupt Threshold: " << mag.getIntThreshold() << "\n";
    }

    if (sensorsActivated.bmp)
    {
        config << "\nBMP Config:\n";
    }

    return config.str();
}
