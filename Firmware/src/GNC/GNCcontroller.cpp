#include "GNCcontroller.h"

void GNCcontroller::setup() {
    setpoint_first << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; //在这里写setpoint
    setpoint_second << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; //在这里写setpoint


    pid1.setup(setpoint_first);
    // pid2.setup(setpoint_second);
}

void GNCcontroller::start() {
    sendArmingCommands();
    pid1.reset();
    // pid2.reset();
}

void GNCcontroller::update(Eigen::Matrix<float,1, 6> currentInput){
    if (millis() - m_previousSampleTime >= m_actuationDelta) {
        input_first = currentInput; 
        pid1.update(input_first);
        output_first = pid1.getOutputValues();

        // pid2.update(input_second);
        
        sendActuationCommands(output_first);
    }
}
void GNCcontroller::update(Eigen::Matrix<float,1, 6> currentInput, float scaling_factor){
    if (millis() - m_previousSampleTime >= m_actuationDelta) {
        input_first = currentInput; 
        pid1.update(input_first);
        output_first = pid1.getOutputValues();
        output_first(0,2) = 1;
        output_first(0,3) = 1;
        output_first(0,2) *= scaling_factor;
        output_first(0,3) *= scaling_factor;
        sendActuationCommands(output_first);
        // pid2.update(input_second);       
    }
}

void GNCcontroller::stop() {
    changeServoAngle(0,0);
    changeServoAngle(1,0);
    sendDisarmingCommands();
}

void GNCcontroller::sendArmingCommands() {
    armServos();
    // armProps();
}

void GNCcontroller::sendDisarmingCommands() {
    disarmServos();
    // disarmProps();
}

void GNCcontroller::sendActuationCommands(Eigen::Matrix<float,1, 4> actuation_values) {
    changeServoAngle(0,actuation_values(0,0)); // pitch servo
    changeServoAngle(1,actuation_values(0,1)); // roll servo
    // changePropPower(0,actuation_values(0,2)); 
    // changePropPower(1,actuation_values(0,3)); 
}

void GNCcontroller::sendActuationCommands(Eigen::Matrix<float,1, 4> actuation_values, float scaling_factor) {
    changeServoAngle(0,actuation_values(0,0)); // pitch servo
    changeServoAngle(1,actuation_values(0,1)); // roll servo
    changePropPower(0,actuation_values(0,2)*scaling_factor); 
    changePropPower(1,actuation_values(0,3)*scaling_factor); 
    
}
void GNCcontroller::armServos() {
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

void GNCcontroller::disarmServos() {
    SimpleCommandPacket arm_alpha(4, 0); //3 here is the arm command
    arm_alpha.header.source_service = 1;
    arm_alpha.header.source = 2;
    arm_alpha.header.destination_service = 10;
    arm_alpha.header.destination = 102;
    arm_alpha.header.uid = 0;
    m_networkmanager.sendPacket(arm_alpha);
    delay(100);
    SimpleCommandPacket arm_beta(4, 0);
    arm_beta.header.source_service = 1;
    arm_beta.header.source = 2;
    arm_beta.header.destination_service = 11;
    arm_beta.header.destination = 102;
    arm_beta.header.uid = 0;
    m_networkmanager.sendPacket(arm_beta);
}

void GNCcontroller::changeServoAngle(int servo, int angle) { // angle should be -10 to 10

    angle = angle * 10; // scale the angle to 0.1 degree = 1 argument degree
    uint8_t des_ser; 

    if (servo == 0) { // alpha mapped from 0 - 400 w 140 as 0
        des_ser = 10; 
        angle += 140;
    }
    if (servo == 1) { // beta mapped from 0 - 400 w 225 as 0
        des_ser = 11; 
        angle += 225;
    }


    angle = round(angle); //round to the nearest integer
    SimpleCommandPacket actuate_servo(2, angle); //2 is the fire command
    actuate_servo.header.source_service = 1;
    actuate_servo.header.source = 2;
    actuate_servo.header.destination_service = des_ser;
    actuate_servo.header.destination = 102;
    actuate_servo.header.uid = 0;
    m_networkmanager.sendPacket(actuate_servo);
}

void GNCcontroller::changePropPower(int prop, int power) {

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

void GNCcontroller::telemetry_impl(packetptr_t packetptr) {
    SimpleCommandPacket packet(*packetptr);

	PIDTelemetryPacket telemetry;

	telemetry.header.type = 108;
	telemetry.header.source = packet.header.destination;
	telemetry.header.source_service = m_serviceID;
	telemetry.header.destination = packet.header.source;
	telemetry.header.destination_service = packet.header.source_service;
	telemetry.header.uid = packet.header.uid; 
	telemetry.pitch_angle = output_first(0,0);
	telemetry.roll_angle = output_first(0,1);
	telemetry.prop_0 = output_first(0,2);
	telemetry.prop_1 = output_first(0,3);

	m_networkmanager.sendPacket(telemetry);
}