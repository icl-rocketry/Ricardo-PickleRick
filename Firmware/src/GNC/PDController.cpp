#include "GNC/PDController.h"
#include <algorithm>
#include <cmath>

void PDController::setup()
{
    // Desired thrust direction in WORLD frame
    // Change this to match your world convention.
    m_thrust_dir_world_des << 0.0f, 0.0f, -1.0f;

    // m_rEng is COM -> thrust centre in body frame (m). Tune ry/rz for attitude-only lateral drift:
    // with rx < 0, negative body-y drift -> make ry more negative; positive body-y drift -> make ry more positive.
    // negative body-z drift -> make rz more negative; positive body-z drift -> make rz more positive.
    // m_rEng << -0.235f, -0.0007f, 0.015f; //centre of mass to center of thrust in body frame
    m_rEng << -0.235f, -0.006f, 0.011f;
    m_mass = 1.32f;

    m_K_p << 0.0f, 2.5f, 2.0f; // attitude body control gains (roll, pitch, yaw)
    m_K_d << 7.0f, 0.8f, 0.9f;

    m_K_p_pos << 0.15f, 0.15f, 0.2f;   // NED position control gains
    m_K_d_pos << 0.8f, 0.8f, 1.0f; 
    m_K_i_pos << 0.0f, 0.0f, 0.01f;
    // m_K_p_pos << 0.0f, 0.0f, 0.3f;   // NED position control gains
    // m_K_d_pos << 0.0f, 0.0f, 1.0f; 
    // m_K_i_pos << 0.0f, 0.0f, 0.04f;


    m_pos_int.setZero();
    m_pos_des << 0.0f, 0.0f, 0.0f;  // need new function to set this externally if you want to move around
    m_vel_des.setZero();
    m_acc_des.setZero();
    m_pos_err_dbg.setZero();
    m_vel_err_dbg.setZero();
    m_euler_error.setZero();
    m_thrust_vector_error_deg.setZero();
    m_max_vel       = 1.0f;                  // m/s — conservative
    m_max_tilt_rad  = MAX_POSITION_TILT_RAD;
    m_last_update_us = 0;

    m_position_control_enabled = true;      // arm explicitly
    m_Fx_cmd_outer  = NOMINAL_FX_N;
}

void PDController::setPositionTarget(const Eigen::Vector3f& position,
                                     const Eigen::Vector3f& velocity,
                                     const Eigen::Vector3f& acceleration)
{
    m_pos_des = position;
    m_vel_des = velocity;
    m_acc_des = acceleration;
}

void PDController::setPositionControlEnabled(bool enabled)
{
    if (!enabled) {
        m_pos_int.setZero();
    }

    m_position_control_enabled = enabled;
}

void PDController::update(Eigen::Quaterniond q, 
                          Eigen::Vector3f angular_rates, 
                          Eigen::Vector3f position, //NED
                          Eigen::Vector3f velocity, //NED
                          float batt_V, bool batt_fresh)
{
    m_batt_V = batt_V;
    m_batt_fresh = batt_fresh;
    q.normalize();
    updatePositionControl(position, velocity);
    updateThrustDirectionErrors(q);
    updateDesiredForce(q);
    updateMcmd(angular_rates);
    updateOutputValues(); 
}

void PDController::reset()
{
    m_dir_error_body << 0.0f, 0.0f, 0.0f;
    m_M_cmd          << 0.0f, 0.0f, 0.0f;
    m_output_values  << 0.0f, 0.0f, 0.0f, 0.0f;
    m_Fx_cmd         = 0.0f;
    m_voltage_scale  = 1.0f;
    m_euler_error.setZero();
    m_thrust_vector_error_deg.setZero();
    m_pos_err_dbg.setZero();
    m_vel_err_dbg.setZero();
}

void PDController::updatePositionControl(const Eigen::Vector3f& position,
    const Eigen::Vector3f& velocity)
{
    // ── dt ───────────────────────────────────────────────────────────────
    const uint32_t now = micros();

    if (m_last_update_us == 0) {

        m_last_update_us = now;
        m_dt = 0.0f;
        return;

    }

    m_dt = (now - m_last_update_us) * 1e-6f;
    m_last_update_us = now;

    if (m_dt <= 0.0f || m_dt > 0.1f) {return;}

    if (!m_position_control_enabled) {

        m_pos_int.setZero();
        m_thrust_dir_world_des << 0.0f, 0.0f, -1.0f;   // -Z is up in World Frame 
        m_Fx_cmd_outer = NOMINAL_FX_N;
        return;

    }

    // ── Simple PID: position error → acceleration command ────────────────
    Eigen::Vector3f pos_err = m_pos_des - position;

    Eigen::Vector3f vel_err = m_vel_des - velocity;

    // Integrator
    m_pos_int += pos_err * m_dt;

    const float I_MAX = 5.0f;
    m_pos_int = m_pos_int.cwiseMax(-I_MAX).cwiseMin(I_MAX);

    Eigen::Vector3f a_des =
    m_acc_des
    + m_K_p_pos.cwiseProduct(pos_err)
    + m_K_i_pos.cwiseProduct(m_pos_int)
    + m_K_d_pos.cwiseProduct(vel_err);

    // Gravity compensation: -Z is up
    a_des += Eigen::Vector3f(0.0f, 0.0f, -GRAVITY);

    // ── Convert acceleration command to thrust vector ────────────────────
    Eigen::Vector3f F_des_world = limitPositionTiltRequest(m_mass * a_des);
    float F_mag = F_des_world.norm();

    if (F_mag < 1e-3f) {
        m_thrust_dir_world_des << 0.0f, 0.0f, -1.0f;
        m_Fx_cmd_outer = 0.0f;
    } else {
        Eigen::Vector3f dir = F_des_world / F_mag;

        m_thrust_dir_world_des = dir;
        m_Fx_cmd_outer = std::clamp(F_mag, 0.0f, MAX_THRUST_N);
    }

    // Telemetry
    m_pos_err_dbg = pos_err;
    m_vel_err_dbg = vel_err;
}

Eigen::Vector3f PDController::limitPositionTiltRequest(const Eigen::Vector3f& force_world) const
{
    Eigen::Vector3f limited_force = force_world;

    const float upward_force = std::max(-limited_force(2), 0.0f);
    limited_force(2) = -upward_force;

    const float max_horizontal_force = upward_force * std::tan(m_max_tilt_rad);
    const float horizontal_force = limited_force.head<2>().norm();

    if (horizontal_force > max_horizontal_force) {
        if (horizontal_force > 1e-6f) {
            limited_force.head<2>() *= max_horizontal_force / horizontal_force;
        } else {
            limited_force.head<2>().setZero();
        }
    }

    return limited_force;
}

void PDController::updateThrustDirectionErrors(const Eigen::Quaterniond& q)
{
    // Body thrust axis = +x_body
    const Eigen::Vector3d thrust_axis_body(1.0, 0.0, 0.0);

    // Current thrust direction in WORLD frame
    Eigen::Vector3d thrust_dir_world = q * thrust_axis_body;
    thrust_dir_world.normalize();

    // Desired thrust direction in WORLD frame
    Eigen::Vector3d thrust_dir_world_des = m_thrust_dir_world_des.cast<double>();
    if (thrust_dir_world_des.norm() < 1e-6) {
        thrust_dir_world_des << 0.0, 0.0, -1.0;
    }
    thrust_dir_world_des.normalize();

    // Error axis in WORLD frame
    // This is zero when the vectors align, and ignores roll about thrust axis
    Eigen::Vector3d e_world = thrust_dir_world.cross(thrust_dir_world_des);

    // Convert error into BODY frame so it matches body rates and actuator axes
    Eigen::Vector3d e_body = q.conjugate() * e_world;

    m_dir_error_body = e_body.cast<float>();

    const Eigen::Vector3d desired_body = q.conjugate() * thrust_dir_world_des;
    //send to telemetry
    const double total_error_rad = std::asin(std::clamp(e_world.norm(), 0.0, 1.0));
    const double pitch_error_rad = std::atan2(desired_body.z(), desired_body.x());
    const double yaw_error_rad   = std::atan2(desired_body.y(), desired_body.x());

    m_thrust_vector_error_deg << static_cast<float>(total_error_rad * RAD_TO_DEG),
                                 static_cast<float>(pitch_error_rad * RAD_TO_DEG),
                                 static_cast<float>(yaw_error_rad * RAD_TO_DEG);

    // No control about thrust axis (body x)
    m_dir_error_body(0) = 0.0f;
}

void PDController::updateMcmd(const Eigen::Vector3f& angular_rates)
{
    Eigen::Vector3f rates_error = -angular_rates; //desired rates are 0 so it is negative
    m_euler_error = m_dir_error_body;//send the errors to telemetry for debugging
    m_dir_error_body(0) = 0.0f; //no roll angle error since its only a D controller
    m_M_cmd =
        -m_K_p.cwiseProduct(m_dir_error_body)
        -m_K_d.cwiseProduct(rates_error);    
}
void PDController::updateDesiredForce(const Eigen::Quaterniond&)
{
    const float Fx_target = m_position_control_enabled ? m_Fx_cmd_outer : NOMINAL_FX_N;
    // Tilt compensation is intentionally disabled; Fx is not increased as attitude error grows.
    m_Fx_cmd = std::clamp(Fx_target, 0.0f, MAX_THRUST_N); //tilt compensation disabled, will lead to better lateral control at the cost of vertical control when tilted
}
void PDController::updateOutputValues()
{
    //take in the centre of mass offsets for the engine 
    const float rx = m_rEng(0);
    const float ry = m_rEng(1);
    const float rz = m_rEng(2);

    //compute desired body forces
    
    Eigen::Vector3f F_body;
    F_body(0) = m_Fx_cmd;
    F_body(1) = (m_M_cmd(2) + ry * m_Fx_cmd) / rx;
    F_body(2) = (rz * m_Fx_cmd - m_M_cmd(1)) / rx;
   
    // Convert desired body forces into servo angles and thrust commands.
    float pitch_servo = -std::atan2(-F_body(2), F_body(0)) * RAD_TO_DEG;
    float yaw_servo   =  std::atan2( F_body(1), F_body(0)) * RAD_TO_DEG;
    float base_thrust = sqrtf(F_body(0)*F_body(0) + F_body(1)*F_body(1) + F_body(2)*F_body(2)) * 100.0f / MAX_THRUST_N;
    
    //--------VOLTAGE SCALING-----------------
    float voltage_scale = 1.0f;
    if (m_batt_fresh && m_batt_V > MIN_VALID_BATT_V)
    {
        const float raw_voltage_scale = NOMINAL_BATT_V / m_batt_V;
        voltage_scale = powf(raw_voltage_scale, VOLTAGE_SCALE_EXPONENT);
        voltage_scale = std::clamp(voltage_scale, MIN_VOLTAGE_SCALE, MAX_VOLTAGE_SCALE);
    }
    m_voltage_scale = voltage_scale;

    base_thrust *= voltage_scale; //scale thrust based on voltage read 

    pitch_servo = std::clamp(pitch_servo, -MAX_GIMBAL_DEG, MAX_GIMBAL_DEG);
    yaw_servo   = std::clamp(yaw_servo,   -MAX_GIMBAL_DEG, MAX_GIMBAL_DEG);
    base_thrust = std::clamp(base_thrust, 0.0f, 100.0f);

    //_--------ROLL CONTROL-----------------
    // Roll-rate damping via differential prop throttle.
    // Positive roll_mix: top CW prop up, bottom CCW prop down.
    float roll_mix = -m_M_cmd(0);
    roll_mix *= voltage_scale; //voltage scaling for roll mix 
    roll_mix += ROLL_MIX_OFFSET; //constant trim to counter negative roll bias
    roll_mix = std::clamp(roll_mix, -MAX_ROLL_MIX, MAX_ROLL_MIX);
   
    float thrust_top = std::clamp(base_thrust + roll_mix, 0.0f, 100.0f);
    float thrust_bottom = std::clamp(base_thrust - roll_mix, 0.0f, 100.0f);

    //--------Thrust Linerisation----------------- 
    thrust_top = 100.0f * powf(thrust_top/ 100.0f, THRUST_EXPONENT);
    thrust_bottom = 100.0f * powf(thrust_bottom / 100.0f, THRUST_EXPONENT);

    //Sending values to telemetry 
    m_roll_mix = roll_mix; //send the roll mix to telemetry for debugging
    m_f_body = F_body; //send the body forces to telemetry for debugging

    m_output_values << pitch_servo,
                       yaw_servo,
                       thrust_top,
                       thrust_bottom;
}
