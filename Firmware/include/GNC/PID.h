#pragma once

#include <Eigen/Dense>
#include <Arduino.h>
 
class PID 
{
    public:

        void setup(Eigen::Matrix<float,1, 6> setpoint);
        void update(Eigen::Matrix<float,1, 6> currentValues);
        void reset();
        Eigen::Matrix<float,1, 4> getOutputValues() { return m_output_values; }; 
    
    private:

        void updateOutputValues();
        void updateErrors(Eigen::Matrix<float,1, 6> currentValues, double dt);

        unsigned long m_previousSampleTime;

        Eigen::Matrix<float,6, 4> m_K_p;
        Eigen::Matrix<float,6, 4> m_K_i;
        Eigen::Matrix<float,6, 4> m_K_d;

        Eigen::Matrix<float,1, 6> m_setpoint;

        Eigen::Matrix<float,1, 6> m_proportional_error;
        Eigen::Matrix<float,1, 6> m_integral_error; 
        Eigen::Matrix<float,1, 6> m_derivative_error; 

        Eigen::Matrix<float,1, 6> m_previous_proportional_error; //update this change later

        Eigen::Matrix<float,1, 4> m_output_values; 

};