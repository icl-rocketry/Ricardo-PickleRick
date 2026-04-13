#include "GNC/PDController.h"

void PDController::setup(){

    m_setpoint << 0,0,0;

    m_rEng << -0.165, 0.0, 0.0;
    m_mass = 1.19;

    m_K_p << 0.0, 0.5, 0.7;
    m_K_d << 0.0, 0.15, 0.2;

}

void PDController::update(Eigen::Matrix<float,1, 7> currentValues){

    int dt_i = millis() - m_previousSampleTime;

    if (dt_i >= 100) {
        Eigen::Quaterniond q(
            currentValues(0),  // w
            currentValues(1),  // x
            currentValues(2),  // y
            currentValues(3)   // z
        );
        Eigen::Vector3f angular_rates(
            currentValues(4), // gx
            currentValues(5), // gy
            currentValues(6)  // gz
        );

        updateQuatErrors(q); 
        updateMcmd(angular_rates); 

        updateOutputValues(q);
        m_previousSampleTime = millis();

    }

}

void PDController::reset() {

    m_quat_error << 0.0, 0.0, 0.0;
    // m_output_values << 0.0, 0.0, 0.0, 0.0;
    m_euler_error << 0.0, 0.0, 0.0;
}

void PDController::updateQuatErrors(Eigen::Quaterniond q){

    Eigen::Quaterniond q_d(1, 0, 0, 0);

    Eigen::Quaterniond q_result = q_d.conjugate() * q;
    float ew = q_result.w(), ex = q_result.x(), ey = q_result.y(), ez = q_result.z();
    float err_roll  = atan2(2*(ew*ex + ey*ez), 1 - 2*(ex*ex + ey*ey));
    float err_pitch = asin(2*(ew*ey - ez*ex));
    float err_yaw   = atan2(2*(ew*ez + ex*ey), 1 - 2*(ey*ey + ez*ez));
    m_euler_error = Eigen::Vector3f(err_roll, err_pitch, err_yaw);
    m_quat_error << q_result.x(), q_result.y(), q_result.z();

}

void PDController::updateMcmd(Eigen::Vector3f angular_rates){

    m_M_cmd =
        - m_K_p.cwiseProduct(m_quat_error)
        - m_K_d.cwiseProduct(angular_rates);
}

void PDController::updateOutputValues(Eigen::Quaterniond q)
{
    double L = m_rEng(0);
    double Fx_ned = 9.5;
    // double Fx_ned = 9.81f * m_mass;

    // Thrust vector in NED frame (along x/north axis)
    Eigen::Vector3d F_ned(Fx_ned, 0.0, 0.0);

    // Rotate thrust into body frame
    Eigen::Vector3d F_body = q.inverse() * F_ned;

    // Add moment-derived forces in body frame
    F_body(1) += -m_M_cmd(2) / L;
    F_body(2) +=  m_M_cmd(1) / L;

    m_f_body = F_body.cast<float>();

    double pitch_servo = -std::atan2(-F_body(2), F_body(0)) * (180.0 / M_PI);
    double yaw_servo   =  std::atan2( F_body(1), F_body(0)) * (180.0 / M_PI);
    double thrust      = F_body.norm() * 100.0 / 22.0;

    m_output_values << static_cast<float>(pitch_servo),
                       static_cast<float>(yaw_servo),
                       static_cast<float>(thrust);
}
