#pragma once

#include <Eigen/Dense>
#include <Arduino.h>
 
class PDController 
{
    public:

        void setup();
        void update(Eigen::Matrix<float,1, 7> currentValues);
        void reset();
        Eigen::Vector4f getOutputValues()   { return m_output_values; }; 
        Eigen::Vector3f getEulerError()     { return m_euler_error; }; 
        Eigen::Vector3f getFBody()          { return m_f_body; }; 
        Eigen::Vector3f getMcmd()           { return m_M_cmd; };
        float getRollMix()   {return m_roll_mix;} ;

    private:
        void updateThrustDirectionErrors(const Eigen::Quaterniond& q);
        void updateMcmd(const Eigen::Vector3f& angular_rates);
        void updateOutputValues();
        void updateDesiredForce(const Eigen::Quaterniond& q);
        unsigned long m_previousSampleTime;

        Eigen::Vector3f m_K_p;
        Eigen::Vector3f m_K_d;
        Eigen::Vector3f m_quat_error;
        Eigen::Vector3f m_M_cmd;
        Eigen::Vector3f m_setpoint;
        Eigen::Vector3f m_rEng;

        Eigen::Vector3f m_euler_error;
        Eigen::Vector3f m_f_body;
        Eigen::Vector3f m_thrust_dir_world_des;
        Eigen::Vector3f m_dir_error_body;
        Eigen::Vector4f m_output_values;

        static constexpr float MAX_GIMBAL_DEG = 15.0f;
        static constexpr float MAX_THRUST_N   = 21.0f;
        static constexpr float MAX_ROLL_MIX   = 8.0f;
        static constexpr float NOMINAL_FX_N = 10.0f;

        float m_Fx_cmd = 0.0f;
        float m_roll_mix;
        float m_mass;



};