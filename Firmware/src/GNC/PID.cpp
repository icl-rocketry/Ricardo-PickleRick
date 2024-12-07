#include <Eigen/Dense>
#include "PID.h"

void PID::setup(){
    createTestK_p(); 
    setpoint << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
    previousError << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    timestep = 0.01; 
    integral_error_riemman << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    integral_error_trapezoid << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}

void PID::update(Eigen::Matrix<float,1, 6> currentPosition){
    updateErrors(currentPosition); 
    sendActuationCommands()
}

void PID::updateErrors(Eigen::Matrix<float,1, 6> currentPosition){

    // Serial.println("Current position: " + String(currentPosition(0,3))); 

    //Proportional Error = setpoint - inputMatrix; 
    for (int i = 0; i < setpoint.cols(); i++) {
        error (0,i) = setpoint(0,i) - currentPosition(0,i);
        }
    // Serial.println("Proportional Error: " + String(error(0,3))); 


    //Integral Error = 
        //Righthand Riemman Sum
    for (int i = 0; i < setpoint.cols(); i++) {
        integral_error_riemman(0,i) += error(0,i)*timestep ; 
        }
    // Serial.println("Int Riemman Error: " + String(integral_error_riemman(0,3))); 

        //Trapezoid Rule
    for (int i = 0; i < setpoint.cols(); i++) {
        integral_error_trapezoid(0,i) += (error(0,i) + previousError(0,i))*timestep*0.5 ; 
        }
    // Serial.println("Int Tra Error: " + String(integral_error_trapezoid(0,3))); 

    //Derivative Error = 
    for (int i = 0; i < setpoint.cols(); i++) {
        derivative_error(0,i) = (error(0,i) - previousError(0,i))/timestep ; 
        }
    // Serial.println("Derivative Error: " + String(derivative_error(0,3))); 

    previousError = error; 
}

Eigen::Matrix<float,1, 4> PID::updateActuationValues(Eigen::Matrix<float,1, 6> inputMatrix){
    return (K_p*error + K_i*integral_error_riemman + K_d*derivative_error)
}

void PID::sendActuationCommands() {

}

void PID::createTestK_p() {
    K_p << 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0;
}

void PID::calibrate_impl(packetptr_t packetptr) { // saves pid gains for all p, i & d matrices
    PIDCalibrationPacket calibrate_comm(*packetptr);
    std::vector<uint8_t> serializedData = packetptr->getBody();
    //should be able to access k_11 k_12 from pidcalibpacket
    //save that into the kp gains matrix
    std::string NVSName = "FRANCIS";
    NVSStore _NVS(NVSName, NVSStore::calibrationType::Servo);
    
    _NVS.saveBytes(serializedData);

    // READ FUN

   
};

void PID::check_gains() {
    std::string NVSName = "FRANCIS";
    NVSStore _NVS(NVSName, NVSStore::calibrationType::Servo);
    //load gains matrix and print it all to see if the calib command can change the values
    std::vector<uint8_t> calibSerialised = _NVS.loadBytes();
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
}

void PID::armServos() {
    SimpleCommandPacket arm_alpha(3, 0);
    test_command_1.header.source_service = 1;
    test_command_1.header.source = 2;
    test_command_1.header.destination_service = 10;
    test_command_1.header.destination = 102;
    test_command_1.header.uid = 0;
    networkmanager.sendPacket(arm_alpha);

    SimpleCommandPacket arm_beta(3, 0);
    test_command_1.header.source_service = 1;
    test_command_1.header.source = 2;
    test_command_1.header.destination_service = 11;
    test_command_1.header.destination = 102;
    test_command_1.header.uid = 0;
    networkmanager.sendPacket(arm_beta);
}

void PID::changeServoAngle(int servo, int angle) {

    servoAngle1 = (servoAngle1) * (180 / 3.1415);
    int32_t sA1 = std::round(servoAngle1);

    SimpleCommandPacket test_command_2(2, sA1);
    test_command_2.header.source_service = 1;
    test_command_2.header.source = 2;
    test_command_2.header.destination_service = 11;
    test_command_2.header.destination = 102;
    test_command_2.header.uid = 0;
    networkmanager.sendPacket(test_command_2);

}

void PID::changePropPower(int prop, int power) {

}