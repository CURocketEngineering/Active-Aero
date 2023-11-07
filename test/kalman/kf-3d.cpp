// #include <iostream>
//#include <Eigen/Dense>

using namespace Eigen;
// using namespace std;

// Define the state vector (position, velocity, acceleration) and measurement vector (altitude, vertical velocity, vertical acceleration)
typedef Matrix<float, 9, 1> StateVector;
typedef Matrix<float, 3, 1> MeasurementVector;

// Define Kalman Filter matrices (A, H, Q, R, P)
Matrix<float, 9, 9> A; // State transition matrix
Matrix<float, 3, 9> H; // Measurement matrix
Matrix<float, 9, 9> Q; // Process noise covariance
Matrix<float, 3, 3> R; // Measurement noise covariance
Matrix<float, 9, 9> P; // Estimate error covariance

StateVector x_hat; // Estimated state

// Initialize Kalman Filter matrices and state vector
void InitializeKalmanFilter()
{
    // Initialize A, H, Q, R, P with appropriate values
    // This depends on the specifics of your system and sensor characteristics

    // Initialize the state vector with initial values
    x_hat << 0, 0, 0, // Initial position (x, y, z)
        0, 0, 0,      // Initial velocity (vx, vy, vz)
        0, 0, 0;      // Initial acceleration (ax, ay, az)

    // Initialize the error covariance matrix P with appropriate values
    // Typically, the initial uncertainty in the state is specified here
}

// Predict the next state using the process model
void Predict()
{
    // Predict the state using the state transition matrix A and the previous state estimate x_hat
    x_hat = A * x_hat;

    // Update the error covariance matrix P using the process noise covariance Q
    P = A * P * A.transpose() + Q;
}

// Update the state estimate based on a new measurement
void Update(const MeasurementVector &measurement)
{
    // Calculate the Kalman gain
    Matrix<float, 9, 3> K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // Update the state estimate based on the measurement and Kalman gain
    x_hat = x_hat + K * (measurement - H * x_hat);

    // Update the error covariance matrix P
    P = (Matrix<float, 9, 9>::Identity() - K * H) * P;
}

int main()
{
    // Initialize Kalman Filter
    InitializeKalmanFilter();

    // Simulate measurements (altitude, vertical velocity, vertical acceleration)
    MeasurementVector measurement;
    measurement << 1000, 50, -9.81; // Example measurement values

    // Kalman Filter loop
    for (int i = 0; i < 10; ++i)
    {
        // Prediction step
        Predict();

        // Update step
        Update(measurement);

        // Print the estimated state (velocity)
        // cout << "Estimated Velocity: " << x_hat.segment(3, 3) << endl;
        Serial.printf("Estimated Velocity: %f", x_hat.segment(3, 3));
    }

    return 0;
}
