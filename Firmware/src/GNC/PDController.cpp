#include "GNC/PDController.h"
#include <algorithm>
#include <cmath>

void PDController::setup()
{
    // Desired thrust direction in WORLD frame
    // Example: point straight up in world-x if that is your vertical axis.
    // Change this to match your world convention.
    m_thrust_dir_world_des << 1.0f, 0.0f, 0.0f;

    m_rEng << -0.23f, -0.005f, 0.003f;
    m_mass = 1.35f;

    m_K_p << 0.0f, 2.0f, 1.5f; 
    m_K_d << 7.0f, 0.45f, 0.5f;

    m_K_p_pos << 0.0f, 0.0f, 0.0f;   // x is "up"; start with vertical off
    m_K_d_pos << 0.0f, 0.0f, 0.0f;
    m_K_i_pos << 0.0f, 0.0f, 0.0f;

    m_pos_int.setZero();
    m_pos_des << 0.0f, 0.0f, 0.0f;  // need new function to set this externally if you want to move around
    m_vel_des.setZero();
    m_acc_des.setZero();
    m_max_vel       = 1.0f;                  // m/s — conservative
    m_max_tilt_rad  = 15.0f * M_PI / 180.0f; // 15° max tilt command
    m_last_update_us = 0;

    m_position_control_enabled = false;      // arm explicitly
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
                          Eigen::Vector3f position, 
                          Eigen::Vector3f velocity,
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
        m_thrust_dir_world_des << 1.0f, 0.0f, 0.0f;   // +X up
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

    // Gravity compensation: +X is up
    a_des += Eigen::Vector3f(GRAVITY, 0.0f, 0.0f);

    // ── Convert acceleration command to thrust vector ────────────────────
    Eigen::Vector3f F_des_world = m_mass * a_des;
    float F_mag = F_des_world.norm();

    if (F_mag < 1e-3f) {
        m_thrust_dir_world_des << 1.0f, 0.0f, 0.0f;
        m_Fx_cmd_outer = 0.0f;
    } else {
        Eigen::Vector3f dir = F_des_world / F_mag;

        // Tilt limit relative to +X up
        const float cos_tilt = dir(0);
        const float cos_max  = std::cos(m_max_tilt_rad);

        if (cos_tilt < cos_max) {
        Eigen::Vector3f horiz(0.0f, dir(1), dir(2));
        const float h_norm = horiz.norm();

        if (h_norm > 1e-6f) {
        horiz *= std::sin(m_max_tilt_rad) / h_norm;
        }

        dir << std::cos(m_max_tilt_rad), horiz(1), horiz(2);
        dir.normalize();
        }

        m_thrust_dir_world_des = dir;
        m_Fx_cmd_outer = std::clamp(F_mag, 0.0f, MAX_THRUST_N);
    }

    // Telemetry
    m_pos_err_dbg = pos_err;
    m_vel_err_dbg = vel_err;
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
        thrust_dir_world_des << 1.0, 0.0, 0.0;
    }
    thrust_dir_world_des.normalize();

    // Error axis in WORLD frame
    // This is zero when the vectors align, and ignores roll about thrust axis
    Eigen::Vector3d e_world = thrust_dir_world.cross(thrust_dir_world_des);

    // Convert error into BODY frame so it matches body rates and actuator axes
    Eigen::Vector3d e_body = q.conjugate() * e_world;

    m_dir_error_body = e_body.cast<float>();

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
void PDController::updateDesiredForce(const Eigen::Quaterniond& q)
{
    const float sin_tilt = std::clamp(m_dir_error_body.norm(), 0.0f, 0.95f);
    const float cos_tilt = std::clamp(std::sqrt(1.0f - sin_tilt * sin_tilt), 0.5f, 1.0f);

    const float Fx_target = m_position_control_enabled ? m_Fx_cmd_outer : NOMINAL_FX_N;
    m_Fx_cmd = std::clamp(Fx_target / cos_tilt, 0.0f, MAX_THRUST_N);
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
        voltage_scale = NOMINAL_BATT_V / m_batt_V;
        voltage_scale = std::clamp(voltage_scale, MIN_VOLTAGE_SCALE, MAX_VOLTAGE_SCALE);
    }

    base_thrust *= voltage_scale; //scale thrust based on voltage read 

    pitch_servo = 0.0;//std::clamp(pitch_servo, -MAX_GIMBAL_DEG, MAX_GIMBAL_DEG);
    yaw_servo   = 0.0;//std::clamp(yaw_servo,   -MAX_GIMBAL_DEG, MAX_GIMBAL_DEG);
    base_thrust = 0.0;//std::clamp(base_thrust, 0.0f, 100.0f);

    //_--------ROLL CONTROL-----------------
    // Roll-rate damping via differential prop throttle.
    // Positive roll_mix: top CW prop up, bottom CCW prop down.
    float roll_mix = m_M_cmd(0);
    roll_mix *= voltage_scale; //voltage scaling for roll mix 
    roll_mix = std::clamp(roll_mix, -MAX_ROLL_MIX, MAX_ROLL_MIX);
   
    float thrust_top = std::clamp(base_thrust + roll_mix, 0.0f, 100.0f);
    float thrust_bottom = std::clamp(base_thrust - roll_mix, 0.0f, 100.0f);

    //Sending values to telemetry 
    m_roll_mix = roll_mix; //send the roll mix to telemetry for debugging
    m_f_body = F_body; //send the body forces to telemetry for debugging

    m_output_values << pitch_servo,
                       yaw_servo,
                       thrust_top,
                       thrust_bottom;
}
