#include "ahrs.h"

void AHRS::begin(float sampleFrequency) {

};

void AHRS::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    //update(gx,gy,gz,ax,ay,az,mx,my,mz);
}


void AHRS::getGravityVector(float *x, float *y, float *z){

}

void AHRS::getQuaternion(float* w, float* x, float* y, float* z){


}

void AHRS::setQuaternion(float w, float x, float y, float z) {


}


float AHRS::getYaw() {
    return 0;
}

float AHRS::getPitch() {
    return 0;
}

float AHRS::getRoll() {
    return 0;
}