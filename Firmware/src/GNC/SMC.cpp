#include <Eigen/Dense>
#include "SMC.h"
#include<cmath>

void PID::setup(){
    createTestK_p(); 
    createTestK_i(); 
    createTestK_d(); //cpp is a sequencial language

    m_setpoint << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    
    m_timestep = 0.01; 
    m_previous_error << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_integral_error_riemman << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_integral_error_trapezoid << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    armServos();
    m_previousSampleTime = millis();
}

void PID::update(Eigen::Matrix<float,1, 6> currentPosition){
    if (millis() - m_previousSampleTime >= m_actuationDelta) {
        updateErrors(currentPosition); 
        updateActuationValues(); 
        // sendActuationCommands();
        m_previousSampleTime = millis();
    }
}

void PID::updateErrors(Eigen::Matrix<float,1, 6> currentPosition){

    // Serial.println("Current position: " + String(currentPosition(0,3))); 

    //Proportional Error = setpoint - inputMatrix; 
    for (int i = 0; i < m_setpoint.cols(); i++) {
        m_error (0,i) = m_setpoint(0,i) - currentPosition(0,i);
        }
    // Serial.println("Proportional Error: " + String(error(0,3))); 


    //Integral Error = 
        //Righthand Riemman Sum
    for (int i = 0; i < m_setpoint.cols(); i++) {
        m_integral_error_riemman(0,i) += m_error(0,i)*m_timestep ; 
        }
    // Serial.println("Int Riemman Error: " + String(integral_error_riemman(0,3))); 

        //Trapezoid Rule
    for (int i = 0; i < m_setpoint.cols(); i++) {
        m_integral_error_trapezoid(0,i) += (m_error(0,i) + m_previous_error(0,i))*m_timestep*0.5 ; 
        }
    // Serial.println("Int Tra Error: " + String(integral_error_trapezoid(0,3))); 

    //Derivative Error = 
    for (int i = 0; i < m_setpoint.cols(); i++) {
        m_derivative_error(0,i) = (m_error(0,i) - m_previous_error(0,i))/m_timestep ; 
        }
    // Serial.println("Derivative Error: " + String(derivative_error(0,3))); 

    m_previous_error = m_error; 
}

void PID::smc(){
    m_derivative_error;
    m_error;

    //define sliding surface
    Eigen::Matrix<float, 1, 3> s = m_derivative_error.block<1, 3>(0, 0) + lambda_smc.cwiseProduct(m_error.block<1, 3>(0, 0));
    Eigen::Matrix<float, 1, 3> acceleration;
    SMC_x = -eta_x * tanh(sx / psi_x);
    acceleration << eta_smc(0,0)*tanh(s(0,0)/psi_smc(0,0)), eta_smc(0,1)*tanh(s(0,1)/psi_smc(0,1)), eta_smc(0,2)*tanh(s(0,2)/psi_smc(0,2));

    double mass = 1.4;
    double T = mass * sqrt(acceleration(0,0)*acceleration(0,0) + acceleration(0,1)*acceleration(0,1) + (acceleration(0,2) + 9.81)*(acceleration(0,2) + 9.81));
    double Tx = mass * acceleration(0,0);
    double Ty = mass * acceleration(0,1);
    double alpha = asin(-Tx/T);
    double beta = asin(Ty/(T*cos(alpha)));
    actuation_values_smc << T/2,T/2, alpha, beta;

}

void PID::updateActuationValues(){
    m_actuation_values = m_error * m_K_p + actuation_values_smc; // + K_i * integral_error_riemman + K_d * derivative_error; 
    // Serial.println("Actuation Values: " + String(actuation_values(0,0)) + " " + String(actuation_values(0,1)) + " " + String(actuation_values(0,2)) + " " + String(actuation_values(0,3)));
}

void PID::sendActuationCommands() {
    changeServoAngle(0,m_actuation_values(0,0)); // pitch servo
    changeServoAngle(1,m_actuation_values(0,1)); // roll servo
    // changePropPower(0,actuation_values(0,2)); 
    // changePropPower(1,actuation_values(0,3)); 
}

void PID::createTestK_p() {
    m_K_p << 0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 1, 0, 0,
    -1, 0, 0, 0,
    0, 0, 0, 0;
}

void PID::createTestK_i() {
    m_K_i << 0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0;
}

void PID::createTestK_d() {
    m_K_d << 0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0;
}

void PID::createTestLambda() {
    lambda_smc << 0.1, 0.1, 0.1;
}

void PID::createTestEta() {
    eta_smc << 10, 10, 0.1;
}

void PID::createTestPsi() {
    psi_smc << 1, 1, 1;
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
        Serial.println("no data") ;// default NC
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
    m_networkmanager.sendPacket(arm_alpha);
    delay(100);
    SimpleCommandPacket arm_beta(3, 0);
    arm_beta.header.source_service = 1;
    arm_beta.header.source = 2;
    arm_beta.header.destination_service = 11;
    arm_beta.header.destination = 102;
    arm_beta.header.uid = 0;
    m_networkmanager.sendPacket(arm_beta);
}

void PID::changeServoAngle(int servo, int angle) { // angle should be -10 to 10

    angle = angle * 10; // scale the angle to 0.1 degree = 1 argument degree
    uint8_t des_ser; 

    if (servo == 0) { // mapped from 0 - 300 w 185 as 0
        des_ser = 10; 
        angle += 185;
    }
    if (servo == 1) { // mapped from 0 - 300 w 120 as 0
        des_ser = 11; 
        angle += 120;
    }


    angle = round(angle); //round to the nearest integer
    Serial.println("Servo: " + String(servo) + ", Angle: " + String(angle));
    SimpleCommandPacket actuate_servo(2, angle); //2 is the fire command
    actuate_servo.header.source_service = 1;
    actuate_servo.header.source = 2;
    actuate_servo.header.destination_service = des_ser;
    actuate_servo.header.destination = 102;
    actuate_servo.header.uid = 0;
    m_networkmanager.sendPacket(actuate_servo);
}

void PID::changePropPower(int prop, int power) {

    uint8_t des_ser; 

    if (prop == 0) {
         des_ser = 10; 
    }
    if (prop == 1) {
        des_ser = 11; 
    }

    //the power is already between 0 and 100 so no need to change

    SimpleCommandPacket actuate_prop(2, power); //2 is the fire command
    actuate_prop.header.source_service = 1;
    actuate_prop.header.source = 2;
    actuate_prop.header.destination_service = des_ser;
    actuate_prop.header.destination = 103;
    actuate_prop.header.uid = 0; //unknown
    m_networkmanager.sendPacket(actuate_prop);

}

void PID::telemetry_impl(packetptr_t packetptr) {
    SimpleCommandPacket packet(*packetptr);

	PIDTelemetryPacket telemetry;

	telemetry.header.type = 108;
	telemetry.header.source = packet.header.destination;
	telemetry.header.source_service = m_serviceID;
	telemetry.header.destination = packet.header.source;
	telemetry.header.destination_service = packet.header.source_service;
	telemetry.header.uid = packet.header.uid; 
	telemetry.pitch_angle = m_actuation_values(0,0);
	telemetry.roll_angle = m_actuation_values(0,1);
	telemetry.prop_0 = m_actuation_values(0,2);
	telemetry.prop_1 = m_actuation_values(0,3);

	m_networkmanager.sendPacket(telemetry);

}
