#include <Eigen/Dense>
#include "PID.h"
Eigen::Matrix<float,1, 4> outputMat;

Eigen::Matrix<float,1, 4> PID::outputMatrix(Eigen::Matrix<float,1, 6> inputMatrix) {

    Eigen::Matrix<float, 1, 4> outputMat = Eigen::Matrix<float, 1, 4>::Zero();
    for (int j = 0; j < 4; ++j) {
            for (int i = 0; i < 6; ++i) {
                outputMat(0,j) += inputMatrix(0,i) * K_p(i,j);
            }
        }
    return outputMat;
}

void PID::createTestK_p() {
    K_p << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 0, 0;
}

void PID::setup(){
    createTestK_p(); 
    setpoint << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
    previousError << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    timestep = 0.01; 
    integral_error_riemman << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    integral_error_trapezoid << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}

void PID::updateErrors(Eigen::Matrix<float,1, 6> currentPosition){

   Serial.println("Current position: " + String(currentPosition(0,3))); 

    //STEP1: error = setpoint - inputMatrix; 
    for (int i = 0; i < setpoint.cols(); i++) {
        error (0,i) = setpoint(0,i) - currentPosition(0,i);
        }
   Serial.println("Error: " + String(error(0,3))); 

    //STEP2: integral_error = 
        //Righthand Riemman Sum
    for (int i = 0; i < setpoint.cols(); i++) {
        integral_error_riemman(0,i) += error(0,i)*timestep ; 
        }
   Serial.println("Int Riemman Error: " + String(integral_error_riemman(0,3))); 

        //Trapezoid Rule
    for (int i = 0; i < setpoint.cols(); i++) {
        integral_error_trapezoid(0,i) += (error(0,i) + previousError(0,i))*timestep*0.5 ; 
        }
    Serial.println("Int Tra Error: " + String(integral_error_trapezoid(0,3))); 

    previousError = error; 
    Serial.println("Previous Error: " + String(previousError(0,3))); 

}

