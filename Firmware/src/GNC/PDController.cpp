#include "GNC/PDController.h"

void PDController::setup(){

    m_setpoint << 0,0,0;

    m_rEng << 0.0, 0.0, -0.165;
    m_mass = 1.19;

    m_K_p << 0.5, 0.5, 0.5;
    m_K_d << 0.2, 0.2, 0.2;

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
            currentValues(4),
            currentValues(5),
            currentValues(6)
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

}

void PDController::updateQuatErrors(Eigen::Quaterniond q){

    Eigen::Quaterniond q_d(0.413, 0.583, 0.418, -0.561);

    Eigen::Quaterniond q_result = q * q_d;
    m_quat_error << q_result.x(), q_result.y(), q_result.z();
}

void PDController::updateMcmd(Eigen::Vector3f angular_rates){ // rates in radians

    m_M_cmd =
        - m_K_p.cwiseProduct(m_quat_error)
        - m_K_d.cwiseProduct(angular_rates);
}

void PDController::updateOutputValues(Eigen::Quaterniond q)
{
    double L = m_rEng(2);

    double Fx_body = -m_M_cmd(1) / L;
    double Fy_body =  m_M_cmd(0) / L;
    double Fz_ned = 9.81 * m_mass;

    Eigen::Vector3d F_ned(0.0, 0.0, Fz_ned);

    // Eigen::Vector3d Fz_body = q.inverse() * F_ned;
    Eigen::Vector3d F_body(Fx_body, Fy_body, 5.0);

    double phi_xz = std::atan2(F_body(0), F_body(2)) * (180.0 / M_PI);
    double phi_zy = std::atan2(F_body(1), F_body(2)) * (180.0 / M_PI);
    double thrust = F_body.norm() * 100.0 / 22.0;

    m_output_values << static_cast<float>(phi_xz),
                       static_cast<float>(phi_zy),
                       static_cast<float>(thrust);
}
