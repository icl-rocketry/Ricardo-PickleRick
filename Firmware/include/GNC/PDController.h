#pragma once

#include <Eigen/Dense>
#include <Arduino.h>
 
class PDController 
{
    public:

        void setup();
        void update(Eigen::Quaterniond q, 
                    Eigen::Vector3f angular_rates, 
                    Eigen::Vector3f position, 
                    Eigen::Vector3f velocity,
                    float batt_V, bool batt_fresh);
        void reset();
        void setPositionTarget(const Eigen::Vector3f& position,
                               const Eigen::Vector3f& velocity = Eigen::Vector3f::Zero(),
                               const Eigen::Vector3f& acceleration = Eigen::Vector3f::Zero());
        void setPositionControlEnabled(bool enabled);
        Eigen::Vector4f getOutputValues()   { return m_output_values; }; 
        Eigen::Vector3f getEulerError()     { return m_euler_error; }; 
        Eigen::Vector3f getThrustVectorErrorDeg() const { return m_thrust_vector_error_deg; };
        Eigen::Vector3f getFBody()          { return m_f_body; }; 
        Eigen::Vector3f getMcmd()           { return m_M_cmd; };
        Eigen::Vector3f getPositionError()  { return m_pos_err_dbg; };
        Eigen::Vector3f getVelocityError()  { return m_vel_err_dbg; };
        Eigen::Vector3f getDesiredPosition() const { return m_pos_des; };
        Eigen::Vector3f getDesiredVelocity() const { return m_vel_des; };
        Eigen::Vector3f getDesiredAcceleration() const { return m_acc_des; };
        Eigen::Vector3f getDesiredThrustWorld() const { return m_thrust_dir_world_des; };
        float getBatteryVoltage()     const { return m_batt_V; }
        float getVoltageScale()       const { return m_voltage_scale; }
        float getFxCmd()              const { return m_Fx_cmd; }
        float getFxCmdOuter()         const { return m_Fx_cmd_outer; }
        bool getPositionControlEnabled() const { return m_position_control_enabled; }
        float getRollMix()                  { return m_roll_mix; } ;

    private:
        void updatePositionControl(const Eigen::Vector3f& position, const Eigen::Vector3f& velocity);
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
        Eigen::Vector3f m_thrust_vector_error_deg;
        Eigen::Vector3f m_f_body;
        Eigen::Vector3f m_thrust_dir_world_des;
        Eigen::Vector3f m_dir_error_body;
        Eigen::Vector4f m_output_values;

        static constexpr float MAX_GIMBAL_DEG = 15.0f;
        static constexpr float MAX_THRUST_N   = 28.0f;
        static constexpr float MAX_ROLL_MIX   = 8.0f; 
        static constexpr float ROLL_MIX_OFFSET = 2.7f;
        static constexpr float NOMINAL_FX_N = 13.24;

        //battery stuff
        float m_batt_V = 15.6f;//initialise
        bool m_batt_fresh = false;
        float m_voltage_scale = 1.0f;

        static constexpr float NOMINAL_BATT_V = 15.6f; //measured voltage of a fully loaded pack under load
        static constexpr float VOLTAGE_SCALE_EXPONENT = 0.95; //exponent for voltage scaling curve, higher means more aggressive scaling at lower voltages
        static constexpr float MIN_VALID_BATT_V = 12.0f;
        static constexpr float MIN_VOLTAGE_SCALE = 0.8f; 
        static constexpr float MAX_VOLTAGE_SCALE = 1.1f;

        //Thrust linearisation model
        static constexpr float THRUST_EXPONENT = 0.7f; //
        

        float m_Fx_cmd = 0.0f;
        float m_roll_mix = 0.0f; 
        float m_mass;

        // Outer loop gains
        Eigen::Vector3f m_K_p_pos;   // position → velocity setpoint
        Eigen::Vector3f m_K_d_pos;   // 
        Eigen::Vector3f m_K_i_pos;   // 
        Eigen::Vector3f m_pos_int;   // integral term

        // Limits / setpoint
        Eigen::Vector3f m_pos_des;
        Eigen::Vector3f m_vel_des;
        Eigen::Vector3f m_acc_des;
        float m_max_vel;
        float m_max_tilt_rad;
        float m_dt;
        uint32_t m_last_update_us;

        // Outer-loop thrust magnitude (overrides NOMINAL_FX_N when active)
        float m_Fx_cmd_outer;
        bool  m_position_control_enabled;
        static constexpr float GRAVITY = 9.81f;

        // Debug telemetry
        Eigen::Vector3f m_pos_err_dbg;
        Eigen::Vector3f m_vel_err_dbg;

};
