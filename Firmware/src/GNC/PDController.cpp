#include "GNC/PDController.h"
#include <algorithm>
#include <cmath>

void PDController::setup()
{
    // Desired thrust direction in WORLD frame
    // Example: point straight up in world-x if that is your vertical axis.
    // Change this to match your world convention.
    m_thrust_dir_world_des << 1.0f, 0.0f, 0.0f;

    m_rEng << -0.24f, 0.0f, 0.0f;
    m_mass = 1.19f;

    // No roll control about body x
    m_K_p << 0.0f, 1.6f, 1.4f; 
    m_K_d << 0.0f, 0.33f, 0.25f;
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
        updateMcmd(angular_rates);
        updateOutputValues(); 
}

void PDController::reset()
{
    m_dir_error_body << 0.0f, 0.0f, 0.0f;
    m_M_cmd          << 0.0f, 0.0f, 0.0f;
    m_output_values  << 0.0f, 0.0f, 0.0f;
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
    Eigen::Vector3f rates_error = -angular_rates;
    rates_error(0) = 0.0f; //no roll rate damping
    m_euler_error = m_dir_error_body;//send the errors to telemetry for debugging
    m_M_cmd =
        -m_K_p.cwiseProduct(m_dir_error_body)
        -m_K_d.cwiseProduct(rates_error);

    // Explicitly enforce no roll moment command
    m_M_cmd(0) = 0.0f;
}

void PDController::updateOutputValues()
{
    const float L = m_rEng(0);   // likely negative
    Eigen::Vector3f F_body;

    // Set nominal thrust along body +x
    F_body(0) = 5.0f;

    // From M = r x F, with r = [L,0,0]:
    // My = -L*Fz  => Fz = -My/L
    // Mz =  L*Fy  => Fy =  Mz/L
    F_body(1) =  m_M_cmd(2) / L; // Mz gives Fy
    F_body(2) = -m_M_cmd(1) / L; // My gives Fz, with a negative sign because of the direction of the moment arm

    m_f_body = F_body; //send to telemetry for debugging

    double pitch_servo = -std::atan2(-F_body(2), F_body(0)) * (180.0 / M_PI);
    double yaw_servo   =  std::atan2( F_body(1), F_body(0)) * (180.0 / M_PI);
    double thrust      = F_body.norm() * 100.0 / 22.0;

    pitch_servo = std::clamp(pitch_servo, -15.0, 15.0); 
    yaw_servo   = std::clamp(yaw_servo,   -15.0, 15.0);
    thrust      = std::clamp(thrust,       0.0, 100.0);
    
    
    m_output_values << static_cast<float>(pitch_servo),
                       static_cast<float>(yaw_servo), 
                       static_cast<float>(thrust);
}