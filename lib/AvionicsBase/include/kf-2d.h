#ifndef KF2D_H
#define KF2D_H

#include <linalg.h> //matrix declaration is column major
using namespace linalg::aliases;
using namespace linalg;

class KF2D
{
    public:

    // Derivables and measurables
    int n = 3; // Number of state variables, measurable or derivable by equations
    int m = 2; // Number of measurable variables by sensor //will be 3 with pitot or vin --> replace all 2's with 3's

    // Define the state vector (position, velocity, acceleration) and measurement vector (altitude, vertical velocity, vertical acceleration)
    typedef mat<float, 3, 1> StateVector;       // state variable x
    typedef mat<float, 2, 1> MeasurementVector; // state measurement z

    StateVector x_hat; // Estimated state

    // Define Kalman Filter matrices (P, A, H, R, Q)
    mat<float, 3, 3> P; // Estimate error covariance
    mat<float, 3, 3> A; // State transition matrix
    mat<float, 2, 3> H; // State to measurement matrix
    mat<float, 2, 2> R; // Measurement noise covariance matrix
    mat<float, 3, 3> Q; // Process noise covariance matrix

    KF2D(); //constructor
    void InitializeKalmanFilter(float delta_time); //initialize, only needs to run once. can also be used to reset a KF. should be ran at 
    void Predict(); //should I also have a delta time predict?? or take average over time dt and use that 
    void Update(const MeasurementVector& measurement, float delta_time);

};


#endif