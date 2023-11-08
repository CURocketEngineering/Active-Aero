#ifndef AHRS_H
#define AHRS_H
#include "Adafruit_AHRS_FusionInterface.h"
#include "Adafruit_AHRS_NXPFusion.h"

class AHRS : public Adafruit_AHRS_FusionInterface
{
public:
    AHRS(){

    };
    Adafruit_NXPSensorFusion interface;
    //I just noticed that magnetometer and accelerometer XYZ aren't the same are we using raw data or are we accounting for them in telem?

    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
    {
        interface.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    }

    void begin(float sampleFrequency = 115200.0F)
    {
        interface.begin(sampleFrequency);
        interface.setRotationVector(0, 0, 0);//why is the arduino/pio library different per system 
        //this will cause an error for anyone compiling not on my laptop, how can i get this without modifying the library but keeping the interface object???
        //i might not be able to, so the object sync might have to be moved back when i can do that next
    };

    void getRotationVector(float *x, float *y, float *z)
    {
        interface.getRotationVector(x, y, z);
    }

    void setRotationVector(float x, float y, float z)
    {
        interface.setRotationVector(x, y, z);
    }

    float getRoll() { return interface.getRoll(); }
    float getPitch() { return interface.getPitch(); }
    float getYaw() { return interface.getYaw(); }

    void getQuaternion(float *w, float *x, float *y, float *z)
    {
        interface.getQuaternion(w, x, y, z);
    }

    void setQuaternion(float w, float x, float y, float z)
    {
        interface.setQuaternion(w, x, y, z);
    }

    void getLinearAcceleration(float *x, float *y, float *z) const
    {
        interface.getLinearAcceleration(x, y, z);
    } // in g
    void getGravityVector(float *x, float *y, float *z)
    {
        interface.getGravityVector(x, y, z);
    } // in g

    typedef struct
    {
        float q0; // w
        float q1; // x
        float q2; // y
        float q3; // z
    } Quaternion_t;
};

#endif