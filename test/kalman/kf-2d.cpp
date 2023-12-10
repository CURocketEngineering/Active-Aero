// #include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
// using namespace std;

// Derivables and measurables 
int n = 3; // Number of state variables, measurable or derivable by equations
int m = 2; // Number of measurable variables by sensor //will be 3 with pitot

// Define the state vector (position, velocity, acceleration) and measurement vector (altitude, vertical velocity, vertical acceleration)
typedef Matrix<float, n, 1> StateVector; //state variable x
typedef Matrix<float, m, 1> MeasurementVector; //state measurement z

// Define Kalman Filter matrices (P, A, H, R, Q)
Matrix<float, n, n> P; // Estimate error covariance
Matrix<float, n, n> A; // State transition matrix
Matrix<float, m, n> H; // State to measurement matrix
Matrix<float, m, m> R; // Measurement noise covariance matrix 
Matrix<float, n, n> Q; // Process noise covariance matrix

//orthogonalization/diagonalization
//symmetric matrices
//orthogonal diagonalization
//least squares for error

StateVector x_hat; // Estimated state

// Initialize Kalman Filter matrices and state vector
void InitializeKalmanFilter()
{
    // Initialize P, A, H, R, Q with appropriate values //NEEDS TO BE DONE
    // dt will NOT be consistent between updates, so they need to be replaced every update
    // This depends on the specifics of the system and sensor characteristics

    P << _, _, _,
        _, _, _,
        _, _, _;

    A << _, _, _,
        _, _, _,
        _, _, _;

    H << _, _, _,
        _, _, _;

    R << _, _,
        _, _;

    Q << _, _, _,
        _, _, _,
        _, _, _;

    // Initialize the state vector with initial values
    /* x_hat << 0, 0, 0, // Initial position (x, y, z)
        0, 0, 0,      // Initial velocity (vx, vy, vz) //
        0, 0, 0;      // Initial acceleration (ax, ay, az) */
        x_hat << 0, 0, 0; //Initial posititon, velocity, acceleration (y, vy, ay)

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
// Might need another param for the dt for the transformation matrix
// DEFINITELY need another param for the dt for the transformation matrix, UNLESS it can be done locally
// In either case, it needs to be update in state matrices using dt
void Update(const MeasurementVector &measurement)
{
    // Calculate the Kalman gain
    Matrix<float, n, m> K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // Update the state estimate based on the measurement and Kalman gain
    x_hat = x_hat + K * (measurement - H * x_hat);

    // Update the error covariance matrix P
    P = (Matrix<float, n, n>::Identity() - K * H) * P;
}

int main()
{
    // Initialize Kalman Filter
    InitializeKalmanFilter();

    // Simulate measurements (altitude, vertical velocity, vertical acceleration)
    MeasurementVector measurement;
    measurement << 1000, -9.81; // Example measurement values //no V measurement, just pos and acc

    // Kalman Filter loop, wont really be a thing here, only call in main
    for (int i = 0; i < 10; i++)
    {
        // Prediction step
        Predict();

        // Update step
        Update(measurement);

        // Print the estimated state (velocity)
        cout << "State Estimate: " << x_hat.toDenseMatrix() << endl;
        //Serial.printf("Estimated Velocity: %f", x_hat.segment(3, 3));
    }

    return 0;
}
