#include "oli_controller.h"

void Oli_controller::setup(Eigen::Matrix<float,1, 12> m_personal_setpoint){
    
    m_setpoint = m_personal_setpoint;
    
    m_timestep = 0.001; 
    m_previousSampleTime = millis();

    // Rocket physical parameters
    m_mass = 1.142f; // kg
    m_J << 0.012f, 0, 0,
           0, 0.012f, 0,
           0, 0, 0.0256f; // Inertia matrix
    m_dtCtrl = 0.001f; // control period [s]
    m_rEngZ = -0.05f; // distance nozzle ↔ CoM [m]

    // Actuator lag parameters
    m_tauAct = 0.1f;   // s  (time constant)
    m_kAct   = 1.0f;   // s⁻¹ (


    m_dfilterA = 0.9f;

    // Outer Loop translation gains
    m_kPos1         << 1.7, 1.7, 1.4;    // k₁ for x, y, z
    m_kPos2         << 0.7, 0.7, 1.7;    // k₂ for x, y, z
    //sliding mode gains 
    m_lambdaPosOuter<< 8, 8, 8;    // λ  for x, y, z
    m_etaPosOuter    = 0;        // same η for all axes
    m_psiPosOuter   << 5, 5, 0.1;  // ψ  per axis
    //inner Loop gains
    m_kPos3 = 3; // Fz inner loop 1st gain
    m_kPos4 = 3; // Fz inner loop 2nd gain 
    m_lambdaPosInner = 8; // λ for Fz inner loop
    m_etaPosInner = 0; // η for Fz inner loop
    m_psiPosInner = 5; // ψ for Fz inner loop
    // Attitude gains
    m_kAtt1 = 9; // k₁ for attitude control
    m_kAtt2 = 9; // k₂ for attitude control
    m_lambdaAtt = 8; // λ for attitude control
    m_etaAtt = 0; // η for attitude control
    m_psiAtt = 5; // ψ for attitude control

    m_u_act << 0.0f, 0.0f, 0.0f; // actuator output [N] (Fx, Fy, Fz)

    


    // m_setpoint = m_personal_setpoint;
    
    // m_timestep = 0.001; 
    // m_previousSampleTime = millis();

    // // Outer Loop translation gains
    // m_kPos1         << 5, 5, 4;    // k₁ for x, y, z
    // m_kPos2         << 2, 2, 5;    // k₂ for x, y, z
    // m_lambdaPosOuter<< 8, 8, 8;    // λ  for x, y, z
    // m_etaPosOuter    = 0.4;        // same η for all axes
    // m_psiPosOuter   << 5, 5, 0.1;  // ψ  per axis
    // //inner Loop gains
    // m_kPos3 = 10; // Fz inner loop 1st gain
    // m_kPos4 = 10; // Fz inner loop 2nd gain 
    // m_lambdaPosInner = 8; // λ for Fz inner loop
    // m_etaPosInner = 0.5; // η for Fz inner loop
    // m_psiPosInner = 5; // ψ for Fz inner loop
    // // Attitude gains
    // m_kAtt1 = 30; // k₁ for attitude control
    // m_kAtt2 = 30; // k₂ for attitude control
    // m_lambdaAtt = 8; // λ for attitude control
    // m_etaAtt = 1; // η for attitude control
    // m_psiAtt = 5; // ψ for attitude control

    // m_u_act << 0.0f, 0.0f, 0.0f; // actuator output [N] (Fx, Fy, Fz)
}

void Oli_controller::update(Eigen::Matrix<float,1, 12> currentPosition){
    if (millis() - m_previousSampleTime >= m_timestep*1000) {
        updateOutputValues(currentPosition); 
        m_previousSampleTime = millis();
    }
}

void Oli_controller::reset() {
    m_error << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_output_values << 0.0, 0.0, 0.0, 0.0;
    m_previousSampleTime = millis();
}

void Oli_controller::updateOutputValues(Eigen::Matrix<float,1, 12> currentPosition)
{
    using Vec3 = Eigen::Vector3f;
    using Mat3 = Eigen::Matrix3f;

    /* ------------------------------------------------------------------ */
    /* 0.  Local aliases for the state & reference signals                */
    /* ------------------------------------------------------------------ */
    Vec3  x      = currentPosition.segment<3>(0);          // [m]
    Vec3  v      = currentPosition.segment<3>(3);          // [m/s]
    Vec3  eta    = currentPosition.segment<3>(6);          // [φ θ ψ] [deg]
    Vec3  omegaB = currentPosition.segment<3>(9);          // [p q r] [deg/s]

    Vec3  x_d  = m_setpoint.segment<3>(0);          // [m]
    Vec3  v_d  = m_setpoint.segment<3>(3);          // [m/s]
    Vec3  a_d  = m_setpoint.segment<3>(6);          // [m/s²]

    /* ------------------------------------------------------------------ */
    /* 1.  OUTER-LOOP (inertial frame) – desired force F_outer            */
    /* ------------------------------------------------------------------ */
    Vec3 e_pos = x - x_d;
    Vec3 e_vel = v - v_d;

    Vec3 s_pos = e_vel + m_lambdaPosOuter.transpose().cwiseProduct(e_pos);

    Vec3 F_bs_out;
    F_bs_out(0) = m_mass * ( a_d(0)
          - (m_kPos1(0)+m_kPos2(0))*e_vel(0)
          -  (m_kPos1(0)*m_kPos2(0))*e_pos(0) )
        -  e_pos(0);
    F_bs_out(1) = m_mass * ( a_d(1)
          - (m_kPos1(1)+m_kPos2(1))*e_vel(1)
          -  (m_kPos1(1)*m_kPos2(1))*e_pos(1) )
        -  e_pos(1);
    F_bs_out(2) = m_mass * ( a_d(2)
          - (m_kPos1(2)+m_kPos2(2))*e_vel(2)
          -  (m_kPos1(2)*m_kPos2(2))*e_pos(2) )
        -  e_pos(2);

    Vec3 F_smc_out;
    F_smc_out(0) = -m_mass * m_etaPosOuter * std::tanh( s_pos(0)/m_psiPosOuter(0) );
    F_smc_out(1) = -m_mass * m_etaPosOuter * std::tanh( s_pos(1)/m_psiPosOuter(1) );
    F_smc_out(2) = -m_mass * m_etaPosOuter * std::tanh( s_pos(2)/m_psiPosOuter(2) );

    constexpr float g = 9.80665f;                      // [m/s²]
    Vec3 F_outer = F_bs_out + F_smc_out;
    F_outer(2)  += m_mass * g;                         // add gravity

    /* ------------------------------------------------------------------ */
    /* 2.  ATTITUDE REFERENCE from desired force                          */
    /* ------------------------------------------------------------------ */
    float Fx = F_outer(0),  Fy = F_outer(1),  Fz = F_outer(2);

    float phi_d   = 0.0f;
    float theta_d = 0.0f;
    if (std::abs(Fz) >= 1e-6f) {
        phi_d   =  std::atan2( Fy,  Fz);
        theta_d = -std::atan2( Fx,  Fz);
    }
    const float tiltMax = 7.0f * M_PI / 180.0f;
    phi_d   = std::clamp(phi_d,   -tiltMax, tiltMax);
    theta_d = std::clamp(theta_d, -tiltMax, tiltMax);

    Vec3 eta_d(phi_d, theta_d, 0.0f);                  // ψ_d = 0

    /* --- first-order LPF on eta_d_dot (Tustin) ----------------------- */
    static Vec3 eta_prev      = eta_d;
    static Vec3 etaDot_prev   = Vec3::Zero();
    Vec3 raw_dot  = (eta_d - eta_prev) / m_dtCtrl;
    Vec3 eta_d_dot= m_dfilterA * etaDot_prev + (1.0f - m_dfilterA) * raw_dot;
    eta_prev    = eta_d;
    etaDot_prev = eta_d_dot;

    /* ------------------------------------------------------------------ */
    /* 3.  INNER ATTITUDE LOOP – commanded moment M_cmd                   */
    /* ------------------------------------------------------------------ */
    auto angleWrap = [](float a)->float {
        return std::remainder(a, 2.0f*M_PI);    // (-π, π]
    };

    /* 3.1 Euler-angle error & desired body-rates ---------------------- */
    Vec3 e_ang;
    e_ang << angleWrap(eta(0)-eta_d(0)),
             angleWrap(eta(1)-eta_d(1)),
             angleWrap(eta(2)-eta_d(2));

    /* Mapping η̇ → ω_B for Z-Y-X convention --------------------------- */
    auto TinvZYX = [](float phi, float theta)->Mat3 {
        float ct = std::cos(theta);
        float st = std::sin(theta);
        float sp = std::sin(phi);
        float cp = std::cos(phi);
        Mat3 Tinv;
        Tinv << 1,  0,      -st,
                0,  cp,      sp*ct,
                0, -sp,      cp*ct;
        return Tinv / ct;
    };
    Mat3 Tinv_d  = TinvZYX(eta_d(0), eta_d(1));
    Vec3 etaDotB = Tinv_d * eta_d_dot;

    Vec3 e_ang_dot = omegaB - etaDotB;
    Vec3 s_att     = e_ang_dot + m_lambdaAtt * e_ang;

    Vec3 M_bs;
    M_bs(0) = m_J(0,0) * ( -(m_kAtt1+m_kAtt2)*e_ang_dot(0)
               - m_kAtt1*m_kAtt2*e_ang(0) ) - e_ang(0);
    M_bs(1) = m_J(1,1) * ( -(m_kAtt1+m_kAtt2)*e_ang_dot(1)
               - m_kAtt1*m_kAtt2*e_ang(1) ) - e_ang(1);
    M_bs(2) = m_J(2,2) * ( -(m_kAtt1+m_kAtt2)*e_ang_dot(2)
               - m_kAtt1*m_kAtt2*e_ang(2) ) - e_ang(2);

    Vec3 M_smc;
    M_smc(0) = -m_J(0,0) * m_etaAtt * std::tanh( s_att(0)/m_psiAtt );
    M_smc(1) = -m_J(1,1) * m_etaAtt * std::tanh( s_att(1)/m_psiAtt );
    M_smc(2) = -m_J(2,2) * m_etaAtt * std::tanh( s_att(2)/m_psiAtt );

    Vec3 M_cmd = M_bs + M_smc;

    /* ------------------------------------------------------------------ */
    /* 4.  INNER Z-TRANSLATION LOOP                                      */
    /* ------------------------------------------------------------------ */
    float e_vel_z = e_vel(2);
    float e_pos_z = e_pos(2);
    float s_pos_z = e_vel_z + m_lambdaPosInner * e_pos_z;

    float F_bs_in_z =
          m_mass * a_d(2)
        - m_mass * (m_kPos4 + m_kPos3) * e_vel_z
        - m_mass * (m_kPos3 * m_kPos4) * e_pos_z
        -                e_pos_z;

    float F_smc_in_z =
        -m_mass * m_etaPosInner * std::tanh( s_pos_z / m_psiPosInner );

    float Fz_inner = F_bs_in_z + F_smc_in_z + m_mass * g;

    /* ------------------------------------------------------------------ */
    /* 5.  ACTUATOR MAPM_PING  → body-frame thrust vector u = [Fx Fy Fz]    */
    /*     Lever arm is along -Z, so L = -rEng(3).  (Positive length)     */
    /* ------------------------------------------------------------------ */
    float L  = -m_rEngZ;                         // > 0
    float Fx_cmd = -M_cmd(1) / L;
    float Fy_cmd =  M_cmd(0) / L;
    float Fz_cmd =  Fz_inner;

    /* Clip as in MATLAB ------------------------------------------------ */
    Fx_cmd = std::clamp(Fx_cmd, -90.0f,  90.0f);
    Fy_cmd = std::clamp(Fy_cmd, -90.0f,  90.0f);
    Fz_cmd = std::clamp(Fz_cmd,   0.0f, 100.0f);

    /* ------------------------------------------------------------------ */
    /* 6.  1st-ORDER ACTUATOR LAG (simple Euler step)                     */
    /* ------------------------------------------------------------------ */
    Vec3 u_cmd( Fx_cmd, Fy_cmd, Fz_cmd );

    Vec3 e_u       = m_u_act - u_cmd;
    Vec3 u_dot_cmd = -(1.0f/m_tauAct + m_kAct) * e_u;   // tauAct, kAct ∈ P

    m_u_act += m_dtCtrl * u_dot_cmd;

    /* ------------------------------------------------------------------ */
    /* 7.  Publish – pack to [Fx Fy Fz dummy] just like original header   */
    /* ------------------------------------------------------------------ */
    // m_output_values << m_u_act(0), m_u_act(1), m_u_act(2), 0.0f;
    // return;
    /* ------------------------------------------------------------------ */
    /* 8.  Convert thrust vector → gimbal angles + total thrust           */
    /*     ─  x-z plane  (rotate about body-Y) :  φ_xz = atan2(Fx, Fz)    */
    /*     ─  z-y plane  (rotate about body-X) :  φ_zy = atan2(Fy, Fz)    */
    /*     ─  total magnitude                  :  |F|                     */
    /* ------------------------------------------------------------------ */
    constexpr float RAD2DEG = 180.0f / 3.14159265358979323846f;

    // angles in radians
    float phi_xz = std::atan2(m_u_act(0), m_u_act(2));   // X–Z plane
    float phi_zy = std::atan2(m_u_act(1), m_u_act(2));   // Z–Y plane
    float thrust = m_u_act.norm();                     // total thrust

    // convert to degrees if you prefer
    phi_xz *= RAD2DEG*10;
    phi_zy *= RAD2DEG*10;

    /* ------------------------------------------------------------------ */
    /* 9.  Pack output:  [φ_xz  φ_zy  |F|  spare]                         */
    /* ------------------------------------------------------------------ */
    m_output_values << phi_xz, phi_zy, phi_d, theta_d;
}


Eigen::Matrix<float,1, 4> Oli_controller::getOutputValues() {
    return m_output_values;
}


