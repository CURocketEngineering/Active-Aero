#include <Arduino.h>

#include "apogeeprediction.h"
#include "math.h"

ApogeePrediction::ApogeePrediction(double rocketMass, double dragCoefficent, double crossArea, double targetApogee) : rocketMass(rocketMass), dragCoefficent(dragCoefficent), crossArea(crossArea), targetApogee(targetApogee) {
    currentVelocity = 0;
    lastRecTime = 0;
    predApogee = 0;
}

void ApogeePrediction::eulerFromQuaternion(double *euler, double x, double y, double z, double w) {
    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y * y);
    double roll_x = atan2(t0, t1);
    double t2 = +2.0 * (w * y - z * x);
    t2 = + t2 > +1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    double pitch_y = asin(t2);
    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y * y + z * z);
    double yaw_z = atan2(t3, t4);

    euler[0] = roll_x;
    euler[1] = pitch_y;
    euler[2] = yaw_z;

    // Convert to degrees if needed
    // roll_x = roll_x * 180 / 3.14159
    // pitch_y = pitch_y * 180 / 3.14159
    // yaw_z = yaw_z * 180 / 3.14159
}

// void ApogeePrediction::calcVelocity(double acceleration) {
//     double curTime = micros();
//     double deltaTime = (curTime - lastRecTime) / pow(10, 6);
//     currentVelocity += acceleration * deltaTime;
//     lastRecTime = curTime;
// }

double ApogeePrediction::getVertAccel(double* acceleration, double* euler) {
    // double euler[3] = {0, 0, 0};
    // eulerFromQuaternion(euler, orientation[0], orientation[1], orientation[2], orientation[3]);  // Convert to radians

    double azvect = -acceleration[0] * sin(euler[1]) + acceleration[1]*cos(euler[1])*sin(euler[0]) + acceleration[2] * cos(euler[1])*cos(euler[0]); // vertical acceleration component, from the bno08x
    return azvect;
}

double ApogeePrediction::predictApogee(double velocity, double pressure, double temperature, double altitude) {
    currentVelocity = velocity;
    double rho = pressure*100/(287.058*(temperature+273.15)); // Get dry air density
    // Serial.println(rho);
    double k = 0.5*rho*dragCoefficent*crossArea;
    // Serial.println(k);
    double predApogee = ((rocketMass/(2*k))*log((rocketMass*9.81 + k*pow(currentVelocity,2))/(rocketMass*9.81))+altitude); // Apogee prediction in meters

    return predApogee;
}