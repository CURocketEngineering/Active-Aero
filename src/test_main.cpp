#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>

#include "../lib/ActiveAero/include/apogeeprediction.h"
#include "../lib/Avionics/include/flightstatus.h"
#include "../lib/Avionics/include/kf-2d.h"
#include "../lib/Avionics/include/linalg.h"

FlightStatus flightStatus(16);
KF2D::MeasurementVector measurement;
KF2D KF;
struct KFData {
    float acceleration;
    float velocity;
    float drift;
 };

float rocketMass = 17.23; // need to calculate wet and dry mass of rocket, make two vars
float dragCoefficent = 0.8; // need to calculate drag coefficient of rocket without flaps
float crossArea = 0.02725801; // need to calculate crossarea of rocket without flaps, as well as a new var with flaps extended


float servoAngle; // servo angle global
float predictedApogee = 550;
bool coast = false;
unsigned long startCoastTime;
double previousPredictedApogee;

double totalFlapArea = 0.015;
int angles[] = {0, 30, 60, 90};
int numAngles = 4;
double areas[] = {0, 1.0/3.0 * totalFlapArea, 2.0/3.0 * totalFlapArea, totalFlapArea};
double desiredApogee = 581.22;
double predApogee;

float prevAltitude = 0;
float prevTime = 0;
bool firstLine = true;
bool firstDataLine = true; // To handle first data row correctly
float velocity;

double t4 = 0;
double t3 = 0;
double t2 = 0;
double t1 = 0;

int main() {
    measurement = {1.8154, 10.1503}; //want y then ay
    KF.InitializeKalmanFilter(measurement);


    std::ifstream file("AA2022_train_1.csv"); // Change to your file's actual name
    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    std::string line;
    bool firstLine = true;

    while (std::getline(file, line)) {
        if (firstLine) {  // Skip the header line
            firstLine = false;
            continue;
        }
    
        std::stringstream ss(line);
    
        // Variables for each column
        float time, accelx, accely, accelz, gx, gy, gz;
        float magx, magy, magz, altitude, pressure, temp;
        char comma; // To handle comma delimiters
    
        // Read values from CSV
        if (ss >> time >> comma >> accelx >> comma >> accely >> comma >> accelz >> comma 
                >> gx >> comma >> gy >> comma >> gz >> comma 
                >> magx >> comma >> magy >> comma >> magz >> comma 
                >> altitude >> comma >> pressure >> comma >> temp) {
    
            // Calculate velocity if not the first data line
            if (!firstDataLine) {
                printf("alt %f - prevAlt %f\n", altitude, prevAltitude);
                float deltaAltitude = altitude - prevAltitude;
                printf("time %f - prevTime %f\n", time, prevTime);
                float deltaTime = time - prevTime;
                if (deltaTime != 0) {
                    velocity = deltaAltitude / (deltaTime / 1000);
                    std::cout << "Velocity: " << velocity << " m/s" << std::endl;
                } else {
                    std::cout << "Warning: Zero time difference, skipping velocity calculation." << std::endl;
                }
            }
    
            // Print current data
            std::cout << "Time: " << time 
                      << ", AccelX: " << accelx 
                      << ", AccelY: " << accely 
                      << ", AccelZ: " << accelz 
                      << ", GyroX: " << gx 
                      << ", GyroY: " << gy 
                      << ", GyroZ: " << gz 
                      << ", MagX: " << magx 
                      << ", MagY: " << magy 
                      << ", MagZ: " << magz 
                      << ", Altitude: " << altitude 
                      << ", Pressure: " << pressure 
                      << ", Temp: " << temp 
                      << std::endl;
    
            // Store current values for next iteration
            prevAltitude = altitude;
            prevTime = time;
            firstDataLine = false; // Allow velocity calculation from the second row onwards
    
        } else {
            std::cerr << "Error reading row: " << line << std::endl;
        }
        flightStatus.newTelemetry(accelx, altitude);
        std::cout<<flightStatus.getStageString()<<std::endl;

        double euler[3];
        euler[0] = atan2(gy, gz);
        euler[1] = atan2(-gx, sqrt(gy * gy + gz * gz));
        euler[2] = 0;

        double accel[] = {accelx, accely, accelz};
        //printf("Accel: x = %f, y = %f, z = %f\n", accel[0], accel[1], accel[2]);
        double vAccel = ApogeePrediction::getVertAccel(accel, euler);
        //printf("vAccel %f\n", vAccel);

        measurement = {(float)altitude, (float)(vAccel)};

        KF.Update(measurement);
        //KF.Predict();
        KFData kfData = {
            acceleration: KF.x_hat[2],
            velocity: KF.x_hat[1],
            drift: KF.x_hat[0]
        };
        //printf("KF State: Pos = %f, Vel = %f, Accel = %f\n", KF.x_hat[0], KF.x_hat[1], KF.x_hat[2]);
        double predApogee;


        if (flightStatus.getStage() == ARMED
            || flightStatus.getStage() == ASCENT
            || flightStatus.getStage() == APOGEE
            || flightStatus.getStage() == DESCENT
            || flightStatus.getStage() == ONGROUND)
        {
            servoAngle = 0;
        }
        else if (flightStatus.getStage() == COAST)
        {
            if (altitude < desiredApogee - (0.15 * desiredApogee)) {
                printf("Level 1: aim for %f\n", desiredApogee + (0.1 * desiredApogee));
                double minDistance = 100000;
                int setting = 0;
                for (int i = 0; i < numAngles; ++i) {
                    previousPredictedApogee = ApogeePrediction::predictApogeeWithFlaps(velocity, altitude, pressure, temp, dragCoefficent, rocketMass, crossArea, areas[i]);
                    printf("Predicted Apogee: %f\n", previousPredictedApogee);
                    if (previousPredictedApogee >= desiredApogee + (0.1 * desiredApogee)
                        && previousPredictedApogee - desiredApogee + (0.1 * desiredApogee) <= minDistance) 
                    {
                        minDistance = previousPredictedApogee;
                        setting = i;
                    }
                    else { break; }
                }
                printf("\n");
                printf("%i\t%f\n\n", angles[setting], previousPredictedApogee);
            }
            else if (altitude < desiredApogee - (0.05 * desiredApogee)) {
                printf("Level 2: aim for %f\n", desiredApogee + (0.05 * desiredApogee));
                double minDistance = 100000;
                int setting = 0;
                for (int i = 0; i < numAngles; ++i) {
                    previousPredictedApogee = ApogeePrediction::predictApogeeWithFlaps(velocity, altitude, pressure, temp, dragCoefficent, rocketMass, crossArea, areas[i]);
                    printf("Predicted Apogee: %f\n", previousPredictedApogee);

                    if (previousPredictedApogee >= desiredApogee + (0.05 * desiredApogee)
                        && previousPredictedApogee - desiredApogee + (0.05 * desiredApogee) <= minDistance) 
                    {
                        minDistance = previousPredictedApogee;
                        setting = i;
                    }
                    else { break; }
                }
                printf("\n");
                printf("%i\t%f\n\n", angles[setting], previousPredictedApogee);
            }
            else if (altitude < desiredApogee) {
                printf("Level 3 (Precision): aim for %f\n", desiredApogee);
                double minDistance = 100000;
                int setting = 0;
                for (int i = 0; i < numAngles; ++i) {
                    previousPredictedApogee = ApogeePrediction::predictApogeeWithFlaps(velocity, altitude, pressure, temp, dragCoefficent, rocketMass, crossArea, areas[i]);
                    printf("Predicted Apogee: %f\n", previousPredictedApogee);

                    printf("%f %f %f %f %f\n", t4, t3, t2, t1, previousPredictedApogee);
                    if (t4 > 0) {previousPredictedApogee = (t4 + t3 + t2 + t1 + previousPredictedApogee) / 5;}

                    if (previousPredictedApogee >= desiredApogee
                        && previousPredictedApogee - desiredApogee <= minDistance) 
                    {
                        minDistance = previousPredictedApogee;
                        setting = i;
                    }
                    else { 
                        break; }
                }
                printf("\n");
                printf("%i\t%f\n\n", angles[setting], previousPredictedApogee);
            }
            t4 = t3;
            t3 = t2;
            t2 = t1;
            t1 = previousPredictedApogee;
            printf("%f %f %f %f\n\n", t4, t3, t2, t1);
        }
    }
    file.close();
    return 0;
}