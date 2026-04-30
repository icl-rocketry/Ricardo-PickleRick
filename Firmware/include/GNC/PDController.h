#pragma once

#include <Eigen/Dense>
#include <Arduino.h>
 
class PDController 
{
    public:

        void setup();
        void update(Eigen::Matrix<float,1, 7> currentValues, float batt_V, bool batt_fresh);
        void reset();
        Eigen::Vector4f getOutputValues()   { return m_output_values; }; 
        Eigen::Vector3f getEulerError()     { return m_euler_error; }; 
        Eigen::Vector3f getFBody()          { return m_f_body; }; 
        Eigen::Vector3f getMcmd()           { return m_M_cmd; };
        float getBatteryVoltage() const { return m_batt_V; }
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
        static constexpr float MAX_ROLL_MIX   = 0.0f;
        static constexpr float NOMINAL_FX_N = 10.0f;

        //battery stuff
        float m_batt_V = 16.8f;
        bool m_batt_fresh = false;

        static constexpr float NOMINAL_BATT_V = 16.8f;
        static constexpr float MIN_VALID_BATT_V = 12.0f;
        static constexpr float MAX_VOLTAGE_SCALE = 1.2f;
        

        float m_Fx_cmd = 0.0f;
        float m_roll_mix = 0.0f;
        float m_mass;

};