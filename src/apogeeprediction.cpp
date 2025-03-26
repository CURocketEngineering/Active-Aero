//#include <Arduino.h>

#include "apogeeprediction.h"
#include "math.h"

#include <iostream>

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

//mass of rocket - rocketMass (kg)
//drag co - dragCoefficient ()
//cross sectional area - crossArea (m^2)
//air density --> pressure (Pa), temperature (C)
// cV - m/s
//A - m
double ApogeePrediction::newPredictApogee(float currentVelocity, float altitude, float pressure, 
float temperature, float dragCoefficient, float rocketMass, float crossArea){
    //std::cout<<currentVelocity<<","<<altitude<<","<<pressure<<","<<temperature<<std::endl;
    //
    //DC: 0.68
    //2.132kg for 1
    //1.75kg for 2
    //CSA: 0.00535205381m^2
    //make these ^^ class values 
    //double dragCoefficient = 0.68;
    //double rocketMass = 1.75;
    //double crossArea = 0.00535205381;

    //green within 10
    //yellow within 50
    //red within 100


    double rho = pressure*100/(287.058*(temperature+273.15)); // Get dry air density
    double k = 0.5*rho*dragCoefficient*crossArea;
    double predApogee = ((rocketMass/(2*k))*log((rocketMass*9.807 + k*pow(currentVelocity,2))/(rocketMass*9.807))+altitude); // Apogee prediction in meters
    return predApogee;
}


double ApogeePrediction::predictApogeeWithFlaps
    (float currentVelocity, float altitude, float pressure, float temperature, 
     float dragCoefficient, float rocketMass, float crossArea, float flapArea) 
{
    //m^2
    double newCrossArea = crossArea + flapArea;
    printf("New cross area %f\n", newCrossArea);

    //flap area needs to be in m^2
    //Dry air density
    printf("pres %f, temp %f\n", pressure, temperature);
    double rho = pressure * 100 / (287.058 * (temperature + 273.15));
    printf("rho %f\n", rho);

    //new
    //(q) [Pa] calculated with dry air density
    double dynamicPressure = 0.5 * rho * pow(currentVelocity, 2); 
    printf("dyanmic pressure %f\n", dynamicPressure);

    //(F_4) [N]
    double forceOnRocket = dynamicPressure * newCrossArea; //here
    printf("force on rocket %f\n", forceOnRocket);


    double newDragCoefficient = dragCoefficient + (2 * forceOnRocket) / (rho * pow(currentVelocity, 2) * newCrossArea);
    printf("new drag c %f\n", newDragCoefficient);
    printf("...Check this brutha %f\n", (2 * forceOnRocket) / (rho * pow(currentVelocity, 2) * newCrossArea));

    double k = 0.5 * rho * newDragCoefficient * newCrossArea;
    printf("k %f\n", k);
    //apogee prediction in meters
    double predApogee = ((rocketMass/(2*k))*log((rocketMass*9.807 + k*pow(currentVelocity,2))/(rocketMass*9.807))+altitude);
    printf("pred apogee %f\n\n", predApogee);
    return predApogee;
}

// trueRocketAltitude = localElevation + rocketAltitude

// temperatureAtRocket = groundAirTemperature - (dryAirTemperatureLapseRate * trueRocketAltitude)

// dryAirDensity = airDensityAtSeaLevel * ((((temperatureAtRocket / standardAirTemperatureAtSeaLevel)) ^ (((gravity * molarMassDryAir)/(specificGasConstantForWaterVapor * dryAirTemperatureLapseRate)) - 1)))

// //saturationVaporPressure = 0.61078 * EXP((17.27 * (temperatureAtRocket - 273.15)) / ((temperatureAtRocket - 273.15) + 237.3)) * 1000

// //partialPressureOfWaterVapor = saturationVaporPressure * relativeAirHumidity * 0.01

// //currentAtmosphericPressure = seaLevelAtmosphericPressure * (1 - ((dryAirTemperatureLapseRate * trueRocketAltitude) / standardAirTemperatureAtSeaLevel)) ^ ((gravity * molarMassOfDryAir) / (specificGasConstantForDryAir * dryAirTemperatureLapseRate))

// //humidAirDensity = dryAirDensity * (1 - (partialPressureOfWaterVapor / currentAtmosphericPressure) * (1 - (molarMassOfWaterVapor / molarMassOfDryAir)))

// //dynamicPressure = (0.5 * humidAirDensity * (rocketVelocity ^ 2)) // / 6895

// dynamicPressure = 0.5 dryAirDensity * (rocketVelocity ^ 2)

// forceOnFourFlaps = dynamicPressure * 4 * oneFlapArea

// newDragCoefficient = dragCoefficient + (2 * forceOnFourFlaps) / (humidAirDensity * (rocketVelocity ** 2) * (4 * oneFlapArea))

// newCrossArea = crossArea + (4 * oneFlapArea)

//90 degrees
//options for servo angle
//7 [0, 15, 30, 45, 60, 75, 90]
//6 [0, 18, 36, 54, 72, 90]
//4 [0, 30, 60, 90]

//flap area
//0 = no deployment
//90 = max deployment area
//hard code flap areas to remove calculations

//only calculate dryAirDensity to remove calculations?
//decide which servo angles to predict apogee with first