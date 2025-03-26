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
double desiredApogee = 500;
double predApogee;

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

    auto now = std::chrono::high_resolution_clock::now();

    // Convert to milliseconds since epoch
    auto previousTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()).count();

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
            
            // Print each variable
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
        printf("Accel: x = %f, y = %f, z = %f\n", accel[0], accel[1], accel[2]);
        double vAccel = ApogeePrediction::getVertAccel(accel, euler);
        printf("vAccel %f\n", vAccel);

        measurement = {(float)altitude, (float)(vAccel)};
        auto now = std::chrono::high_resolution_clock::now();

        // Convert to milliseconds since epoch
        auto nowTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()).count();
        float dt = previousTime - nowTime;
        previousTime = nowTime;
        KF.Update(measurement);
        //KF.Predict();
        KFData kfData = {
            acceleration: KF.x_hat[2],
            velocity: KF.x_hat[1],
            drift: KF.x_hat[0]
        };
        printf("KF State: Pos = %f, Vel = %f, Accel = %f\n", KF.x_hat[0], KF.x_hat[1], KF.x_hat[2]);
        double predApogee;


        if (flightStatus.getStage() == ARMED
            || flightStatus.getStage() == ASCENT
            || flightStatus.getStage() == APOGEE
            || flightStatus.getStage() == DESCENT
            || flightStatus.getStage() == ONGROUND)
        {
            servoAngle = 0;
            previousPredictedApogee = ApogeePrediction::newPredictApogee(KF.x_hat[1], altitude, pressure, temp, dragCoefficent, rocketMass, crossArea);
            printf("Predicted Apogee: %f\n\n", previousPredictedApogee);
        }
        else if (flightStatus.getStage() == COAST)
        {
            if (altitude < desiredApogee - 100) {
                previousPredictedApogee = ApogeePrediction::newPredictApogee(KF.x_hat[1], altitude, pressure, temp, dragCoefficent, rocketMass, crossArea);
                printf("Previous version Predicted Apogee: %f\n", previousPredictedApogee);
                double max = 0;
                int setting = 0;
                for (int i = 0; i < numAngles; ++i) {
                    previousPredictedApogee = ApogeePrediction::predictApogeeWithFlaps(KF.x_hat[1], altitude, pressure, temp, dragCoefficent, rocketMass, crossArea, areas[i]);
                    printf("Predicted Apogee: %f\n", previousPredictedApogee);
                    if (previousPredictedApogee <= desiredApogee + 100
                        && max <= previousPredictedApogee) 
                    {
                        max = previousPredictedApogee;
                        setting = i;
                    }
                }
                printf("\n");
                //printf("%i\t%d\n\n", angles[setting], max);
                
            }
        }
    //         if (!coast) {
    //             coast = true;
    //             auto now = std::chrono::high_resolution_clock::now();

    //             // Convert to milliseconds since epoch
    //             auto startCoastTime = std::chrono::duration_cast<std::chrono::milliseconds>(
    //               now.time_since_epoch()).count();;
    //         }
    //         else {
    //             auto now = std::chrono::high_resolution_clock::now();

    //             // Convert to milliseconds since epoch
    //             auto currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(
    //                     now.time_since_epoch()).count();
    //             //1.5 second after coast starts
    //             //if (startCoastTime - currentTime > 0) {
    //                 if (altitude < desiredApogee - 200) {
    //                     double max = 0;
    //                     int setting = 0;
    //                     for (int i = 0; i < numAngles; ++i) {
    //                         previousPredictedApogee = ApogeePrediction::predictApogeeWithFlaps(KF.x_hat[1], altitude, pressure, temp, dragCoefficent, rocketMass, crossArea, areas[i]);
    //                         if (previousPredictedApogee <= desiredApogee + 100
    //                             && max <= previousPredictedApogee) 
    //                         {
    //                             max = previousPredictedApogee;
    //                             setting = i;
    //                         }
    //                     }
    //                     std::cout<<"Here 2";
    //                     //printf("%i\t%d\n", angles[setting], max);
    //                 }
    //                 else {
    //                     double min = 10000;
    //                     int setting = 0;
    //                     for (int i = 0; i < numAngles; ++i) {
    //                         previousPredictedApogee = ApogeePrediction::predictApogeeWithFlaps(KF.x_hat[1], altitude, pressure, temp, dragCoefficent, rocketMass, crossArea, areas[i]);
    //                         printf("%d", previousPredictedApogee);
    //                         std::cout<<KF.x_hat[1]<<std::endl;
    //                         if (abs(desiredApogee - previousPredictedApogee) <= min) {
    //                             min = abs(desiredApogee - previousPredictedApogee);
    //                             setting = i;
    //                         }
    //                     }
    //                     std::cout<<"Here 3";
    //                     //printf("%i\t%d\n", angles[setting], min);
    //                 }
    //             //}
    //             // else {
    //             //     servoAngle = 0;
    //             //     previousPredictedApogee = ApogeePrediction::newPredictApogee(KF.x_hat[1], altitude, pressure, temp, dragCoefficent, rocketMass, crossArea);
    //             //     std::cout<<"Here 4";
    //             //     printf("%i\t%d\n", 0, 0);
    //             // }
    //         }
    //     }
    // }



    }
    file.close();
    return 0;
}