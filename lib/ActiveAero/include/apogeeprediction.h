#ifndef APOGEEPRED_H
#define APOGEEPRED_H

class ApogeePrediction {
  private:
    const double rocketMass;
    const double dragCoefficent;
    const double crossArea; // In m^2
    const double targetApogee;

    double predApogee;
    double currentVelocity;
    double lastRecTime;
    
    void eulerFromQuaternion(double *euler, double x, double y, double z, double w);
    void calcVelocity(double acceleration);
  public:
    ApogeePrediction(double rocketMass, double dragCoefficent, double crossArea, double targetApogee);
    double getVertAccel(double* acceleration, double* euler);
    double predictApogee(double velocity, double pressure, double temperature, double altitude);
};

#endif