#ifndef AHRS_H
#define AHRS_H
#include "Adafruit_AHRS_FusionInterface.h"
#include "Adafruit_AHRS_NXPFusion.h"

class AHRS : public Adafruit_AHRS_FusionInterface
{
public:
    AHRS(){

    };
    Adafruit_NXPSensorFusion fusor;

    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
    {
        //Serial.println("update called...");
        fusor.update(gx, gy, gz, ax, ay, az, mx, my, mz);
        RVecPl[0] = fusor.RVecPl[0];
        RVecPl[1] = fusor.RVecPl[1];
        RVecPl[2] = fusor.RVecPl[2];
    }
    //* fQuaternionFromRotationVectorDeg(Quaternion_t *pq, const float rvecdeg[], float fscaling)
    // fQuaternionFromRotationMatrix(float R[][3], Quaternion_t *pq)
    // fRotationMatrixFromQuaternion(float R[][3],const Quaternion_t *pq)
    // fRotationVectorDegFromRotationMatrix(float R[][3], float rvecdeg[])
    //* fRotationVectorDegFromQuaternion(Quaternion_t *pq, float rvecdeg[])

    void begin(float sampleFrequency = 115200.0F)
    {
        this->sampleFrequency = sampleFrequency;
        setRotationVector(0, 0, 0);
        fusor.begin(sampleFrequency);
    };

    void getRotationVector(float *x, float *y, float *z)
    {
        *x = RVecPl[0];
        *y = RVecPl[1];
        *z = RVecPl[2];
    }

    void setRotationVector(float x, float y, float z)
    {
        RVecPl[0] = x;
        RVecPl[1] = y;
        RVecPl[2] = z;
    }

    float getRoll() { return PhiPl; }
    float getPitch() { return ThePl; }
    float getYaw() { return PsiPl; }

    void getQuaternion(float *w, float *x, float *y, float *z)
    {
        *w = qPl.q0;
        *x = qPl.q1;
        *y = qPl.q2;
        *z = qPl.q3;
    }

    void setQuaternion(float w, float x, float y, float z)
    {
        qPl.q0 = w;
        qPl.q1 = x;
        qPl.q2 = y;
        qPl.q3 = z;
    }

    void getLinearAcceleration(float *x, float *y, float *z) const
    {
        *x = aSePl[0];
        *y = aSePl[1];
        *z = aSePl[2];
    } // in g
    void getGravityVector(float *x, float *y, float *z)
    {
        *x = gSeGyMi[0];
        *y = gSeGyMi[1];
        *z = gSeGyMi[2];
    } // in g

    typedef struct
    {
        float q0; // w
        float q1; // x
        float q2; // y
        float q3; // z
    } Quaternion_t;

private:
    int sampleFrequency;
    float PhiPl; // roll (deg)
    float ThePl; // pitch (deg)
    float PsiPl; // yaw (deg)
    float RhoPl; // compass (deg)
    float ChiPl; // tilt from vertical (deg)
    // orientation matrix, quaternion and rotation vector
    float RPl[3][3];  // a posteriori orientation matrix
    Quaternion_t qPl; // a posteriori orientation quaternion
    float RVecPl[3];  // rotation vector
    // angular velocity
    float Omega[3]; // angular velocity (deg/s)
    // systick timer for benchmarking
    int32_t systick; // systick timer;
    // end: elements common to all motion state vectors

    // elements transmitted over bluetooth in kalman packet
    float bPl[3];     // gyro offset (deg/s)
    float ThErrPl[3]; // orientation error (deg)
    float bErrPl[3];  // gyro offset error (deg/s)
    // end elements transmitted in kalman packet

    float dErrGlPl[3]; // magnetic disturbance error (uT, global frame)
    float dErrSePl[3]; // magnetic disturbance error (uT, sensor frame)
    float aErrSePl[3]; // linear acceleration error (g, sensor frame)
    float aSeMi[3];    // linear acceleration (g, sensor frame)
    float DeltaPl;     // inclination angle (deg)
    float aSePl[3];    // linear acceleration (g, sensor frame)
    float aGlPl[3];    // linear acceleration (g, global frame)
    float gErrSeMi[3]; // difference (g, sensor frame) of gravity vector (accel)
                       // and gravity vector (gyro)
    float mErrSeMi[3]; // difference (uT, sensor frame) of geomagnetic vector
                       // (magnetometer) and geomagnetic vector (gyro)
    float gSeGyMi[3];  // gravity vector (g, sensor frame) measurement from gyro
    float
        mSeGyMi[3];           // geomagnetic vector (uT, sensor frame) measurement from gyro
    float mGl[3];             // geomagnetic vector (uT, global frame)
    float QvAA;               // accelerometer terms of Qv
    float QvMM;               // magnetometer terms of Qv
    float PPlus12x12[12][12]; // covariance matrix P+
    float K12x6[12][6];       // kalman filter gain matrix K
    float Qw12x12[12][12];    // covariance matrix Qw
    float C6x12[6][12];       // measurement matrix C
    float RMi[3][3];          // a priori orientation matrix
    Quaternion_t Deltaq;      // delta quaternion
    Quaternion_t qMi;         // a priori orientation quaternion
    float casq;               // FCA * FCA;
    float cdsq;               // FCD * FCD;
    float Fastdeltat;         // sensor sampling interval (s) = 1 / SENSORFS
    float deltat;             // kalman filter sampling interval (s) = OVERSAMPLE_RATIO /
                              // SENSORFS
    float deltatsq;           // fdeltat * fdeltat
    float QwbplusQvG;         // FQWB + FQVG
    int8_t
        FirstOrientationLock; // denotes that 9DOF orientation has locked to 6DOF
    int8_t resetflag;         // flag to request re-initialization on next pass
};

#endif