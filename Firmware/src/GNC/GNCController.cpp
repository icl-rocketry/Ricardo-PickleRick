#include "GNC/GNCController.h"

void GNCController::setup() {

    m_pd.setup();

}

void GNCController::start() {

    m_controller_start_time = millis();
    sendArmingCommands();
    m_pd.reset();
    
}

void GNCController::setBatteryVoltage(float batt_V, bool fresh)
{
    m_batt_V = batt_V;
    m_batt_fresh = fresh;
}

void GNCController::update(Eigen::Quaterniond q, 
                           Eigen::Vector3f angular_rates, 
                           Eigen::Vector3f position, 
                           Eigen::Vector3f velocity,
                           bool actuate)
{

    if (millis() - m_previousSampleTime >= m_actuationDelta) {

        m_previousSampleTime = millis();
        m_pd.update(q, angular_rates, position, velocity, m_batt_V, m_batt_fresh);
        m_output = m_pd.getOutputValues();
        if (actuate) {

            sendActuationCommands(m_output);

        }

    }

}

void GNCController::stop() {

    changeServoAngle(0,0);
    changeServoAngle(1,0);
    changePropPower(0,0);
    changePropPower(1,0);
    sendDisarmingCommands();

}

void GNCController::sendActuationCommands(Eigen::Vector4f actuation_values) {

    float max_prop_power = 55.0f;

    float thrust_top = actuation_values(2); 
    float thrust_bottom = actuation_values(3);

    if (thrust_top > max_prop_power) { thrust_top = max_prop_power; }
    if (thrust_bottom > max_prop_power) { thrust_bottom = max_prop_power; }

    changePropPower(0, (int)thrust_top); 
    changePropPower(1, (int)thrust_bottom); 

    float pitch_angle = actuation_values(0); 
    float yaw_angle = actuation_values(1);

    changeServoAngle(1, -pitch_angle); //top servo
    changeServoAngle(0, yaw_angle); //bottom servo
    // changeServoAngle(0, pitch_angle); //top servo <- this config is for if the board is rotated
    // changeServoAngle(1, yaw_angle); //bottom servo
}


// prop 10 is +y
// prop 11 is -z
void GNCController::changeServoAngle(int servo, float angle_f) { // angle should be -20 to 20
    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Changing Servo Angle: " + std::to_string(angle_f));

    float max_commanded_angle = 15.0f;
    float min_commanded_angle = -15.0f;

    if (angle_f > max_commanded_angle) { angle_f = max_commanded_angle; } 
    else if (angle_f < min_commanded_angle) {
        angle_f = min_commanded_angle;
    }

    angle_f = angle_f * 10; // scale the angle to 0.1 degree = 1 argument degree
    int angle = static_cast<int>(angle_f); // convert to int
    uint8_t des_ser; 

    if (servo == 0) { 
        des_ser = 10; 
        angle += 1070;
    }
    if (servo == 1) { 
        des_ser = 11; 
        angle += 920;
    }

    SimpleCommandPacket actuate_servo(2, angle); //2 is the fire command
    actuate_servo.header.source_service = 1;
    actuate_servo.header.source = 2;
    actuate_servo.header.destination_service = des_ser;
    actuate_servo.header.destination = 102;
    actuate_servo.header.uid = 0;
    m_networkmanager.sendPacket(actuate_servo);
}

void GNCController::changePropPower(int prop, int power) {
    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Changing Prop Power: " + std::to_string(power));

    uint8_t des_ser; //10 is +y, 11 is -z
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

void GNCController::sendArmingCommands() {

    armProps();
    armServos();

}
#include <libriccore/riccorelogging.h>

void GNCController::armProps() {
    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Arming Props");
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

void GNCController::armServos() {
    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Arming Serovs");
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

void GNCController::sendDisarmingCommands() {

    disarmServos();
    disarmProps();

}

void GNCController::disarmProps() {
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

void GNCController::disarmServos() {
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

// #include <libriccore/riccoresystem.h>

void GNCController::telemetry_impl(packetptr_t packetptr) {
    SimpleCommandPacket packet(*packetptr);
    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Telemetry_impl called");

	ControllerTelemetryPacket telemetry;

    auto euler_angles = m_pd.getEulerError();
    auto f_body = m_pd.getFBody();
	telemetry.header.type = 108;
	telemetry.header.source = packet.header.destination;
	telemetry.header.source_service = m_serviceID;
	telemetry.header.destination = packet.header.source;
	telemetry.header.destination_service = packet.header.source_service;
	telemetry.header.uid = packet.header.uid; 

    telemetry.q0 =               m_input(0,0);
    telemetry.q1 =               m_input(0,1);
    telemetry.q2 =               m_input(0,2);
    telemetry.q3 =               m_input(0,3);

    telemetry.roll_error =       euler_angles(0) * (180.0f / 3.14159f); // convert to degrees
    telemetry.pitch_error =      euler_angles(1) * (180.0f / 3.14159f); // convert to degrees
    telemetry.yaw_error =        euler_angles(2) * (180.0f / 3.14159f); // convert to degrees

    telemetry.roll_rate_input =  m_input(0,4);
    telemetry.pitch_rate_input = m_input(0,5);
    telemetry.yaw_rate_input =   m_input(0,6);

    telemetry.fx_body =          f_body(0);
    telemetry.fy_body =          f_body(1);
    telemetry.fz_body =          f_body(2);

	telemetry.pitch_output =     m_output(0);
	telemetry.yaw_output =       m_output(1);
	telemetry.thrust_top =       m_output(2);
    telemetry.thrust_bottom=     m_output(3);

    telemetry.m_cmd_y = m_pd.getMcmd()(1); //get the yaw moment command for telemetry
    telemetry.m_cmd_z = m_pd.getMcmd()(2); //get the pitch moment command for telemetry
    telemetry.m_roll_mix = m_pd.getRollMix(); //get the roll mix for telemetry
    telemetry.m_batt = m_pd.getBatteryVoltage(); //get the battery voltage for telemetry
    
    telemetry.system_time = millis();
	m_networkmanager.sendPacket(telemetry);

}