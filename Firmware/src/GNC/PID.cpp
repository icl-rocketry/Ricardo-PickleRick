#include <Eigen/Dense>
#include "PID.h"



void PID::createTestK_p() {
    K_p << 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
}


void PID::save_pid_gains() {
    std::string NVSName = "FRANCIS";
    NVSStore _NVS(NVSName, NVSStore::calibrationType::Servo);
        //save to nvs store
    PIDcalibrationpacket PIDCalibration;
    std::vector<uint32_t> serializedData = PIDCalibraion.getBody();
        _NVS.saveBytes(serializedData);

// READ FUN

    std::vector<uint32_t> calibSerialised = _NVS.loadBytes();
    if(calibSerialised.size() == 0)
    {
        // setNormalState(0); // default is nominally closed
        Serial.println('no data') ;// default NC
        return;
    }
    Serial.println("PID data:");
    // setNormalState(calibpacket.normalState);
   // Display the matrix
for (size_t j = 0; j < 4; ++j)
{
    for (size_t i = 0; i < 6; ++i)
    {
        Serial.print(calibSerialised[j * 6 + i]);// j starts from 0, so first row is [0,x] so 0*6 = 6, plus the i can get the column number
        Serial.print("  "); //space between te data
    }
    Serial.println();// back to the loop
}
};

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

