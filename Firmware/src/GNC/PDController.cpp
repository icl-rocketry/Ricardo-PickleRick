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
    m_mass = 1.19f;

    // // No roll control about body x
    m_K_p << 0.0f, 2.0f, 1.5f; 
    // m_K_p << 0.0f, 0.0f, 0.0f; 
    m_K_d << 7.0f, 0.45f, 0.5f;
   // m_K_d << 0.0f, 0.00f, 0.0f;
}

void PDController::update(Eigen::Matrix<float,1,7> currentValues)
{
     // 100Hz update rate
        Eigen::Quaterniond q(
            currentValues(0),  // w
            currentValues(1),  // x
            currentValues(2),  // y
            currentValues(3)   // z
        );
        q.normalize();

        Eigen::Vector3f angular_rates(
            currentValues(4), // p
            currentValues(5), // q
            currentValues(6)  // r
        );

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
    // Ignore roll-rate damping too, because no roll authority
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

    m_Fx_cmd = std::clamp(NOMINAL_FX_N / cos_tilt, 0.0f, MAX_THRUST_N);
}
void PDController::updateOutputValues()
{
    const float rx = m_rEng(0);
    const float ry = m_rEng(1);
    const float rz = m_rEng(2);
    
    Eigen::Vector3f F_body;
    F_body(0) = m_Fx_cmd;
    F_body(1) = (m_M_cmd(2) + ry * m_Fx_cmd) / rx;
    F_body(2) = (rz * m_Fx_cmd - m_M_cmd(1)) / rx;
   

    float pitch_servo = -std::atan2(-F_body(2), F_body(0)) * RAD_TO_DEG;
    float yaw_servo   =  std::atan2( F_body(1), F_body(0)) * RAD_TO_DEG;
    float base_thrust = sqrtf(F_body(0)*F_body(0) + F_body(1)*F_body(1) + F_body(2)*F_body(2)) * 100.0f / MAX_THRUST_N;

    pitch_servo = std::clamp(pitch_servo, -MAX_GIMBAL_DEG, MAX_GIMBAL_DEG);
    yaw_servo   = std::clamp(yaw_servo,   -MAX_GIMBAL_DEG, MAX_GIMBAL_DEG);
    base_thrust = std::clamp(base_thrust, 0.0f, 100.0f);

    //_--------ROLL CONTROL-----------------
    // Roll-rate damping via differential prop throttle.
    // Positive roll_mix: top CW prop up, bottom CCW prop down.
    float roll_mix = m_M_cmd(0);
    roll_mix = std::clamp(roll_mix, -MAX_ROLL_MIX, MAX_ROLL_MIX);
   
    const float thrust_top = std::clamp(base_thrust + roll_mix, 0.0f, 100.0f);
    const float thrust_bottom = std::clamp(base_thrust - roll_mix, 0.0f, 100.0f);

    //Sending values to telemetry 
    m_roll_mix = roll_mix; //send the roll mix to telemetry for debugging
    m_f_body = F_body; //send the body forces to telemetry for debugging

    m_output_values << pitch_servo,
                       yaw_servo,
                       thrust_top,
                       thrust_bottom;
}


