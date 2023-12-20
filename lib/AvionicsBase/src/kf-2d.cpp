#include "kf-2d.h"

// orthogonalization/diagonalization
// symmetric matrices
// orthogonal diagonalization
// least squares for error

KF2D::KF2D() {} // default constructor, don't want a static KF

// Initialize Kalman Filter matrices and state vector
void KF2D::InitializeKalmanFilter()
{
    
    lastTime = -1;


    // Initialize P, A, H, R, Q with appropriate values //NEEDS TO BE DONE
    // dt will NOT be consistent between updates, so they need to be replaced every update
    // This depends on the specifics of the system and sensor characteristics

    // P is symmetric, no need to col maj (for now)
    P = {{1, 1, 1},
         {1, 1, 1},
         {1, 1, 1}};

    /* //this is what it SHOULD look like written out by hand, but linalg is col maj
    A = {{1,    delta_time,    0.5 * delta_time * delta_time},
         {0,    1,             delta_time},
         {0,    0,             1}};
         */

    A = {{1, 0, 0},
         {0, 1, 0},
         {0, 0, 1}};

    /* //this is what it SHOULD look like written out by hand, but linalg is col maj
       H = {{1.0, 0.0, 0.0},
           {0.0, 0.0, 1.0}};
           */
    H = {{1, 0}, {0, 0}, {0, 1}};

    // R is symmetric, no need to col maj (for now)
    R = {{0.1, 0},
         {0, 0.1}}; // will be 0.1,0,0 ; 0,0.1,0 ; 0,0,0.1 in 3d, or whatever R float instead of 0.1

    // Q is symmetric, no need to col maj (for now)
    Q = {{0.1, 0.1, 0.1},
         {0.1, 0.1, 0.1},
         {0.1, 0.1, 0.1}};

    // Initialize the state vector with initial values
    x_hat = {0,0,0}; // Initial posititon, velocity, acceleration (y, vy, ay)
}

// Predict the next state using the process model
void KF2D::Predict()
{
    // Predict the state using the state transition matrix A and the previous state estimate x_hat
    x_hat = mul(A, x_hat);

    // Update the error covariance matrix P using the process noise covariance Q
    P = mul(mul(A, P), transpose(A)) + Q;
}
#include <Arduino.h>
// Update the state estimate based on a new measurement
// Might need another param for the dt for the transformation matrix
// DEFINITELY need another param for the dt for the transformation matrix, UNLESS it can be done locally
//great news it could be done locally
// In either case, it needs to be update in state matrices using dt
void KF2D::Update(const MeasurementVector &measurement)
{

    auto thisTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    float delta_time = (thisTime - lastTime)/1000; //convert to seconds
    lastTime = thisTime;
    Serial.printf("\tdt=%f\t",delta_time);
     if(lastTime == -1) { //skip the first update by init delta time
          return;
     }

    

    A = {{1, 0, 0},
         {delta_time, 1, 0},
         {(float)(0.5 * delta_time * delta_time), delta_time, 1}};

    // Calculate the Kalman gain
    mat<float, 3, 2> K = mul(mul(P, transpose(H)), inverse(mul(mul(H, P), transpose(H)) + R)); //n,m

    // Update the state estimate based on the measurement and Kalman gain
    x_hat = x_hat + mul(K, (measurement - mul(H, x_hat)));

    // Update the error covariance matrix P
    float3x3 m = linalg::identity;
    P = mul((m - mul(K, H)), P);
}
