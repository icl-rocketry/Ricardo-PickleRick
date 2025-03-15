#include "GNCcontroller.h"

void GNCcontroller::setup(){
    setpoint_first << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; //在这里写setpoint
    setpoint_second << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; //在这里写setpoint

    create_input_first();
    create_angle_error();
    create_input_second(); 
    create_output_second(); 

    // create_m_first(); 
    // create_m_second(); 

    create_m_first_PID_config(); 
    create_m_second_PID_config(); 

    pid1.setup(setpoint_first, m_first_PID_config);
    // networkmanager.registerService(static_cast<uint8_t>(Services::ID::PID), pid1.getThisNetworkCallback());
    pid2.setup(setpoint_second, m_second_PID_config);
    // networkmanager.registerService(static_cast<uint8_t>(Services::ID::PID), pid2.getThisNetworkCallback());

}

void GNCcontroller::update(Eigen::Matrix<float,1, 6> currentInput){
    input_first = currentInput; 
    // first_PID_caculation_matrix(); 
    first_PID_caculation_PID(); 
    angle_error_to_input_second(); 
    // second_PID_caculation_matrix(); 
    second_PID_caculation_PID(); 
}
    //整个操作过程包括：
        // setup(不循环), 
        // update(进input)
        // first caculation
        // angle error to input second
        // second PID caculation
        // get output(不循环)

// void GNCcontroller::first_PID_caculation_matrix() {
//     angle_error = input_first * m_first; 
// }

void GNCcontroller::first_PID_caculation_PID() {
    //body work
    pid1.update(input_first);
    angle_error = pid1.getControllerError(); 
}

void GNCcontroller::angle_error_to_input_second() {
    input_second(1,0) = input_first(1,4)-angle_error(1,0); 
    input_second(2,0) = input_first(1,5)-angle_error(2,0); 
}

// void GNCcontroller::second_PID_caculation_matrix() {
//     output_second = input_second * m_second; 
// }

void GNCcontroller::second_PID_caculation_PID() {
    //body work
    pid2.update(input_second);
    output_second = pid2.getControllerError(); 
}

Eigen::Matrix<float,1, 4> GNCcontroller::getOutput() {
    return output_second; 
}

void GNCcontroller::sendActuationCommands() {
    pid2.sendActuationCommands(); 
}

// shall i use a third PID
// to actually move the servo after i finish caculation

void GNCcontroller::create_input_first() {
    input_first << 0, 0, 0, 0, 0, 0;
}
void GNCcontroller::create_angle_error() {
    angle_error << 0, 0, 0, 0;
}
void GNCcontroller::create_input_second() {
    input_second << 0, 0, 0, 0, 0, 0;
}
void GNCcontroller::create_output_second() {
    output_second << 0, 0, 0, 0;
}
// void GNCcontroller::create_m_first() {
//     m_first << 0, 0, 0, 0,
//     0, 0, 0, 0,
//     0, 0, 0, 0,
//     1, 0, 0, 0,
//     0, 1, 0, 0,
//     0, 0, 0, 0;
// }
// void GNCcontroller::create_m_second() {
//     m_second << 1, 0, 0, 0,
//     0, 1, 0, 0,
//     0, 0, 0, 0,
//     0, 0, 0, 0,
//     0, 0, 0, 0,
//     0, 0, 0, 0;
// }
void GNCcontroller::create_m_first_PID_config() {
    m_first_PID_config << 0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 0, 0;
}
void GNCcontroller::create_m_second_PID_config() {
    m_second_PID_config << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0;
}