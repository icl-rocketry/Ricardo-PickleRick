#pragma once

#include <Eigen/Dense>
#include <Arduino.h>
 
class PDController 
{
    public:

        void setup();
        void update(Eigen::Matrix<float,1, 7> currentValues);
        void reset();
        Eigen::Vector3f getOutputValues()   { return m_output_values; }; 
        Eigen::Vector3f getEulerError()     { return m_euler_error; }; 
        Eigen::Vector3f getFBody()          { return m_f_body; }; 
    
    private:

        void updateQuatErrors(Eigen::Quaterniond q);
        void updateMcmd(Eigen::Vector3f angular_rates);
        void updateOutputValues(Eigen::Quaterniond q);

        unsigned long m_previousSampleTime;

        Eigen::Vector3f m_K_p;
        Eigen::Vector3f m_K_d;
        Eigen::Vector3f m_quat_error;
        Eigen::Vector3f m_M_cmd;
        Eigen::Vector3f m_setpoint;
        Eigen::Vector3f m_rEng;

        Eigen::Vector3f m_euler_error;
        Eigen::Vector3f m_f_body;

        Eigen::Vector3f m_output_values;
        float m_mass;



};