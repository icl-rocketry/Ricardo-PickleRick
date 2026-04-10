#include "GNC/PID.h"

void PID::setup(Eigen::Matrix<float,1, 6> setpoint){

    m_setpoint = setpoint;

}

void PID::update(Eigen::Matrix<float,1, 6> currentValues){

    int dt_i = millis() - m_previousSampleTime;

    if (dt_i >= 100) {
        double dt_d = dt_i/1000;

        updateErrors(currentValues, dt_d); 
        updateOutputValues(); 

        m_previousSampleTime = millis();

    }

}

void PID::reset() {

    m_proportional_error << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_integral_error << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_derivative_error << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    
    m_previous_proportional_error << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_output_values << 0.0, 0.0, 0.0, 0.0;

}

void PID::updateErrors(Eigen::Matrix<float,1, 6> currentValues, double dt){

    //Proportional Error = setpoint - inputMatrix; 
    for (int i = 0; i < m_setpoint.cols(); i++) {
        m_proportional_error (0,i) = m_setpoint(0,i) - currentValues(0,i);
    }

    //Integral Error = 
    for (int i = 0; i < m_setpoint.cols(); i++) {
        m_integral_error(0,i) += (m_proportional_error(0,i) + m_previous_proportional_error(0,i))*dt*0.5 ; 
    }

    //Derivative Error = 
    for (int i = 0; i < m_setpoint.cols(); i++) {
        m_derivative_error(0,i) = (m_proportional_error(0,i) - m_previous_proportional_error(0,i))/dt; 
    }

    m_previous_proportional_error = m_proportional_error;
}

void PID::updateOutputValues(){

    m_output_values = m_proportional_error * m_K_p; // + K_i * integral_error + K_d * derivative_error; 

}
