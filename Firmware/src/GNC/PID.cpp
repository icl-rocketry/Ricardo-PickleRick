#include <Eigen/Dense>
#include "PID.h"

void PID::setup(){
    createTestK_p(); 
    createTestK_i(); 
    createTestK_d(); //cpp is a sequencial language
    setpoint << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
    previous_error << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    timestep = 0.01; 
    integral_error_riemman << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    integral_error_trapezoid << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}

void PID::update(Eigen::Matrix<float,1, 6> currentPosition){
    updateErrors(currentPosition); 
    updateActuationValues(); 
    sendActuationCommands();
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
        integral_error_trapezoid(0,i) += (error(0,i) + previous_error(0,i))*timestep*0.5 ; 
        }
    // Serial.println("Int Tra Error: " + String(integral_error_trapezoid(0,3))); 

    //Derivative Error = 
    for (int i = 0; i < setpoint.cols(); i++) {
        derivative_error(0,i) = (error(0,i) - previous_error(0,i))/timestep ; 
        }
    // Serial.println("Derivative Error: " + String(derivative_error(0,3))); 

    previous_error = error; 
}

void PID::updateActuationValues(){
    actuation_values = K_p * error + K_i * integral_error_riemman + K_d * derivative_error; 
}

void PID::sendActuationCommands() {
    changeServoAngle(0,actuation_values(0,0)); 
    changeServoAngle(1,actuation_values(0,1)); 
    changePropPower(0,actuation_values(0,2)); 
    changePropPower(1,actuation_values(0,3)); 
}

void PID::createTestK_p() {
    K_p << 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0;
}

void PID::createTestK_i() {
    K_i << 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0;
}

void PID::createTestK_d() {
    K_d << 0, 0, 0, 0, 0, 0,
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
    SimpleCommandPacket arm_alpha(3, 0); //3 here is the arm command
    arm_alpha.header.source_service = 1;
    arm_alpha.header.source = 2;
    arm_alpha.header.destination_service = 10;
    arm_alpha.header.destination = 102;
    arm_alpha.header.uid = 0;
    networkmanager.sendPacket(arm_alpha);

    SimpleCommandPacket arm_beta(3, 0);
    arm_beta.header.source_service = 1;
    arm_beta.header.source = 2;
    arm_beta.header.destination_service = 11;
    arm_beta.header.destination = 102;
    arm_beta.header.uid = 0;
    networkmanager.sendPacket(arm_beta);
}

void PID::changeServoAngle(int servo, int angle) {

    uint8_t des_ser; 

    if (servo = 0) {
        des_ser = 10; 
    }
    if (servo = 1) {
        des_ser = 11; 
    }

    angle += 100; // chenge a num from -100 to +100 to a num from 0 to 200 that the servo can take

    SimpleCommandPacket actuate_servo(2, angle); //2 is the fire command
    actuate_servo.header.source_service = 1;
    actuate_servo.header.source = 2;
    actuate_servo.header.destination_service = des_ser;
    actuate_servo.header.destination = 102;
    actuate_servo.header.uid = 0;
    networkmanager.sendPacket(actuate_servo);
}

void PID::changePropPower(int prop, int power) {

    uint8_t des_ser; 

    if (prop = 0) {
         des_ser = 10; 
    }
    if (prop = 1) {
        des_ser = 11; 
    }

    //the power is already between 0 and 100 so no need to change

    SimpleCommandPacket actuate_prop(2, power); //2 is the fire command
    actuate_prop.header.source_service = 1;
    actuate_prop.header.source = 2;
    actuate_prop.header.destination_service = des_ser;
    actuate_prop.header.destination = 102;
    actuate_prop.header.uid = 0; //unknown
    networkmanager.sendPacket(actuate_prop);

}