#ifndef AHRS_H
#define AHRS_H
#include "Adafruit_AHRS_FusionInterface.h"

class AHRS : public Adafruit_AHRS_FusionInterface{
   public:
   AHRS() {

   }

    //fQuaternionFromRotationVectorDeg(Quaternion_t *pq, const float rvecdeg[], float fscaling)
    //fQuaternionFromRotationMatrix(float R[][3], Quaternion_t *pq)
    //fRotationMatrixFromQuaternion(float R[][3],const Quaternion_t *pq) 
    //fRotationVectorDegFromRotationMatrix(float R[][3], float rvecdeg[])
    //fRotationVectorDegFromQuaternion(Quaternion_t *pq, float rvecdeg[])

   //talk w MD if he wants more or less than these, or any differences
   //

   void begin(float sampleFrequency);

   void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    
    void setQuaternion(float w, float x, float y, float z); //should set @ start of launch

    //include v in stateUpdate when/if I can get v integrated into the 3D EKF
    //can samples get thrown off if some but not all get changed in a tick? NS sampling thm
    void stateUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz); 


    //true accel from YPR + Axyz
    /*
    float getAccelY();

    float getAccelZ();

    float getAccelX();
    */
    //pass accel struct with x y z 
    //pass pointer struct and update
    //turn into getAccel

    //void getHeading(); //some heading pointer depends on how they want the values, if at all

    void getGravityVector(float* x, float* y, float* z); //not sure how different this is from accelY but the library has support for both so ill provide access to both

    void getQuaternion(float* w, float* x, float* y, float* z);

    //heading - angle of depression or elevation --> yaw --> not yaw

    float getYaw();

    float getPitch();

    float getRoll();
};

#endif