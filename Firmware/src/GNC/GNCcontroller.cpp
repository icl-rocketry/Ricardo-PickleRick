#include "GNCcontroller.h"

void GNCcontroller::setup() {
    setpoint_first << 0.0, 0.0, 0.01, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0; //在这里写setpoint
    setpoint_second << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0; //在这里写setpoint


    oli_controller.setup(setpoint_first);
    // pid2.setup(setpoint_second);
}

void GNCcontroller::start() {
    sendArmingCommands();
    // pid2.reset();
}

void GNCcontroller::update(Eigen::Matrix<float,1, 13> currentInput){
    if (millis() - m_previousSampleTime >= m_actuationDelta) {
        input_first = currentInput.block<1,12>(0,0);
        float ramp_up_value = currentInput(0,12); // Assuming the 13th value is a ramp up value
        if (ramp_up_value > 1.0) {
            ramp_up_value = 1.0;
        }
        oli_controller.update(input_first);
        output_first = oli_controller.getOutputValues();

        // pid2.update(input_second);
        
        sendActuationCommands(output_first, ramp_up_value);
    }
}

void GNCcontroller::stop() {
    changeServoAngle(0,0);
    changeServoAngle(1,0);
    changePropPower(0,0);
    changePropPower(1,0);
    sendDisarmingCommands();
}

void GNCcontroller::sendArmingCommands() {
    armServos();
    armProps();
}

void GNCcontroller::sendDisarmingCommands() {
    disarmServos();
    disarmProps();
}

void GNCcontroller::sendActuationCommands(Eigen::Matrix<float,1, 4> actuation_values, float ramp_up_value) {
    float max_prop_power = 60.0f;
    ramp_up_value = 1.0f;

    if (actuation_values(0,2) > max_prop_power) {

        actuation_values(0,2) = max_prop_power * ramp_up_value; // scale propeller power
        actuation_values(0,3) = max_prop_power * ramp_up_value * 0.965; // scale propeller power

    } else {

        actuation_values(0,2) = actuation_values(0,2) * ramp_up_value; // scale propeller power
        actuation_values(0,3) = actuation_values(0,3) * ramp_up_value * 0.965; // scale propeller power
    }

    changeServoAngle(0,actuation_values(0,0)); // pitch servo
    changeServoAngle(1,actuation_values(0,1)); // roll servo
    changePropPower(0,actuation_values(0,2)); 
    changePropPower(1,actuation_values(0,3)); 
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

void GNCcontroller::changeServoAngle(int servo, float angle_f) { // angle should be -10 to 10
    if (angle_f > 10) {
        angle_f = 10;
    } else if (angle_f < -10) {
        angle_f = -10;
    }

    angle_f = angle_f * 10; // scale the angle to 0.1 degree = 1 argument degree
    int angle = static_cast<int>(angle_f); // convert to int
    uint8_t des_ser; 

    if (servo == 0) { // alpha mapped from 0 - 400 w 140 as 0
        des_ser = 10; 
        angle += 120;
    }
    if (servo == 1) { // beta mapped from 0 - 400 w 225 as 0
        des_ser = 11; 
        angle += 190;
    }

    SimpleCommandPacket actuate_servo(2, angle); //2 is the fire command
    actuate_servo.header.source_service = 1;
    actuate_servo.header.source = 2;
    actuate_servo.header.destination_service = des_ser;
    actuate_servo.header.destination = 102;
    actuate_servo.header.uid = 0;
    m_networkmanager.sendPacket(actuate_servo);
}

void GNCcontroller::armProps() {
    SimpleCommandPacket arm_prop0(3, 0); //3 here is the arm command
    arm_prop0.header.source_service = 1;
    arm_prop0.header.source = 2;
    arm_prop0.header.destination_service = 10;
    arm_prop0.header.destination = 103;
    arm_prop0.header.uid = 0;
    m_networkmanager.sendPacket(arm_prop0);
    delay(100);
    SimpleCommandPacket arm_prop1(3, 0);
    arm_prop1.header.source_service = 1;
    arm_prop1.header.source = 2;
    arm_prop1.header.destination_service = 11;
    arm_prop1.header.destination = 103;
    arm_prop1.header.uid = 0;
    m_networkmanager.sendPacket(arm_prop1);
}

void GNCcontroller::disarmProps() {
    SimpleCommandPacket disarm_prop0(4, 0); //4 here is the disarm command
    disarm_prop0.header.source_service = 1;
    disarm_prop0.header.source = 2;
    disarm_prop0.header.destination_service = 10;
    disarm_prop0.header.destination = 103;
    disarm_prop0.header.uid = 0;
    m_networkmanager.sendPacket(disarm_prop0);
    delay(100);
    SimpleCommandPacket disarm_prop1(4, 0);
    disarm_prop1.header.source_service = 1;
    disarm_prop1.header.source = 2;
    disarm_prop1.header.destination_service = 11;
    disarm_prop1.header.destination = 103;
    disarm_prop1.header.uid = 0;
    m_networkmanager.sendPacket(disarm_prop1);
}

void GNCcontroller::changePropPower(int prop, int power) {
    Serial.println("Changing prop power: " + String(power));
    uint8_t des_ser; 
    if (power < 0) {
        power = 0; // make sure the power is not negative
    }
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

	ControllerTelemetryPacket telemetry;

	telemetry.header.type = 108;
	telemetry.header.source = packet.header.destination;
	telemetry.header.source_service = m_serviceID;
	telemetry.header.destination = packet.header.source;
	telemetry.header.destination_service = packet.header.source_service;
	telemetry.header.uid = packet.header.uid; 
    telemetry.x_input = input_first(0,0);
    telemetry.y_input = input_first(0,1);
    telemetry.z_input = input_first(0,2);
    telemetry.u_input = input_first(0,3);
    telemetry.v_input = input_first(0,4);
    telemetry.w_input = input_first(0,5);
    telemetry.roll_input = input_first(0,6);
    telemetry.pitch_input = input_first(0,7);
    telemetry.yaw_input = input_first(0,8);
    telemetry.roll_rate_input = input_first(0,9);
    telemetry.pitch_rate_input = input_first(0,10);
    telemetry.yaw_rate_input = input_first(0,11);
	telemetry.pitch_output = output_first(0,0);
	telemetry.roll_output = output_first(0,1);
	telemetry.prop_0 = output_first(0,2);
	telemetry.prop_1 = output_first(0,3);

	m_networkmanager.sendPacket(telemetry);
}