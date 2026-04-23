#include "Estimator/ekf.h"

void EKF::setup(const Eigen::Vector3f& gyro_bias, 
                const Eigen::Vector3f& accel_bias, 
                const Eigen::Vector3f& h_accel_bias,
                const Eigen::Vector3f& mag_ref
)
{
    m_x.setZero();

    // Initialise quaternion to identity [1, 0, 0, 0]
    m_x(6) = 1.0f;

    // Seed bias states from calibration
    m_x.segment<3>(10) = accel_bias;
    m_x.segment<3>(13) = gyro_bias;
    m_h_accel_bias  = h_accel_bias;
    m_mag_ref       = mag_ref;

    // Initial covariance — large uncertainty on everything except quaternion
    m_P.setZero();
    m_P.block<3,3>(0,0)   = 100.0f  * Eigen::Matrix3f::Identity();           // position
    m_P.block<3,3>(3,3)   = 10.0f   * Eigen::Matrix3f::Identity();           // velocity
    m_P.block<4,4>(6,6)   = 1.0f    * Eigen::Matrix<float,4,4>::Identity();  // quaternion
    m_P.block<3,3>(10,10) = 1e-8f   * Eigen::Matrix3f::Identity();           // accel bias (bias frozen for now)
    m_P.block<3,3>(13,13) = 1e-8f   * Eigen::Matrix3f::Identity();           // gyro bias (bias frozen for now)

    m_setHome_ref.launch_lat = 515074000;
    m_setHome_ref.launch_lon = -1278000;
    m_setHome_ref.launch_alt = 400.0f;
    m_setHome_ref.launch_pressure    = 101325.0f;
    m_setHome_ref.launch_temperature = 288.15f;
    
}

void EKF::update(   const Eigen::Vector3f gyro, 
                    const Eigen::Vector3f accel, 
                    const Eigen::Vector3f h_accel, 
                    const Eigen::Vector3f mag,
                    const float pressure,
                    const float temperature,
                    const SensorStructs::GPS_t& gps
                )
{
    const uint32_t now = micros();  // use micros not millis for better dt resolution
    const float dt = (m_lastPredictTime == 0) 
    ? 0.0f 
    : static_cast<float>(now - m_lastPredictTime) * 1e-6f;
    m_lastPredictTime = now;
    
    if (dt <= 0.0f || dt > 0.5f) { return; }  // sanity check — skip bad dt
    
    predict(dt, gyro, accel, h_accel);

    updateMag(mag);

    updateLowGAccel(accel);

    updateBaro(pressure, temperature);

    updateGPS(gps);
}

void EKF::setHome(const SensorStructs::home_ref_t& setHome_ref) 
{ 
    m_setHome_ref = setHome_ref; 
    m_x.segment<6>(0).setZero();
};

void EKF::predict(  const float dt, 
                    const Eigen::Vector3f gyro, 
                    const Eigen::Vector3f accel, 
                    const Eigen::Vector3f h_accel
                )
{
    using Mat3  = Eigen::Matrix3f;
    using Mat4  = Eigen::Matrix<float, 4, 4>;
    using Mat43 = Eigen::Matrix<float, 4, 3>;
    using Vec4  = Eigen::Vector4f;

    const Mat3 I3 = Mat3::Identity();

    // ── Attitude update ─────────────────────────────────────────────────────────
    Vec4 q = m_x.segment<4>(6);
    q.normalize();
    const float q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);

    const float wx = gyro(0) - m_x(13);
    const float wy = gyro(1) - m_x(14);
    const float wz = gyro(2) - m_x(15);
    m_angular_rates << wx, wy, wz;
    const float w_norm = m_angular_rates.norm();

    Mat4 Omega;
    Omega <<     0, -wx, -wy, -wz,
                wx,   0,  wz, -wy,
                wy, -wz,   0,  wx,
                wz,  wy, -wx,   0;

    Mat4 F_qq;
    if (w_norm > 1e-9f)
    {
        // exp((dt/2)*omega) [dont need i from e^ix = cosx + isinx for weird maths reasons]
        F_qq =  std::cos(0.5f * w_norm * dt) * Mat4::Identity()
              + std::sin(0.5f * w_norm * dt) * (Omega / w_norm);
    }
    else // to avoid the /0
    {
        F_qq = Mat4::Identity() + 0.5f * dt * Omega;
    }
        
    m_x.segment<4>(6) = F_qq * q;
    m_x.segment<4>(6).normalize();

    // ── Attitude process noise ──────────────────────────────────────────────────

    // jacobian of prediction wrt angular rates
    Mat43 E_q;
    E_q << -q1, -q2, -q3,
            q0, -q3,  q2,
            q3,  q0, -q1,
           -q2,  q1,  q0;
    
    const Eigen::Matrix<float, 4, 3> G_w = 0.5f * dt * E_q;
    m_Q_att = G_w *
              SIGMA_ALPHA.cwiseProduct(SIGMA_ALPHA).asDiagonal() *
              G_w.transpose();
           

    // ── Translation update ───────────────────────────────────────────────────────

    // ── Select accelerometer
    const float accel_norm   = accel.norm();

    if (accel_norm < LOW_G_SATURATION)          // low-g not saturated
    {
        m_acceleration = accel - m_x.segment<3>(10);
    }
    else
    {
        m_acceleration = h_accel - m_h_accel_bias;
    }

    Vec4 q_new = m_x.segment<4>(6);
    const Mat3 R_body_to_ned = Eigen::Quaternionf(q_new(0), q_new(1), q_new(2), q_new(3)).toRotationMatrix();

    const Eigen::Vector3f g_ned(0.0f, 0.0f, -g);

    const Eigen::Vector3f a_ned = R_body_to_ned * (m_acceleration) - g_ned;
    const float dt2 = dt * dt;

    m_x.segment<3>(0) += m_x.segment<3>(3) * dt + 0.5f * a_ned * dt2;  
    m_x.segment<3>(3) += a_ned * dt; 

    // ── Translation process noise ────────────────────────────────────────────────

    const float dt3 = dt2 * dt;
    const float qj  = SIGMA_JERK * SIGMA_JERK;

    Eigen::Matrix<float, 2, 2> Q_sub;
    Q_sub << qj * dt3 / 3.0f,  qj * dt2 / 2.0f,
            qj * dt2 / 2.0f,  qj * dt;

    // Q_trans = kron(Q_sub, I3) — 6×6
    m_Q_trans.setZero();
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
            m_Q_trans.block<3,3>(i*3, j*3) = Q_sub(i,j) * I3;


    // ── Full F and Q matrices (16×16) ─────────────────────────────────────────
    m_F.setZero();
    m_F.block<3,3>(0,0)   = I3;
    m_F.block<3,3>(0,3)   = dt * I3;   // position depends on velocity
    m_F.block<3,3>(3,3)   = I3;        // velocity integrates
    m_F.block<4,4>(6,6)   = F_qq;      // attitude
    m_F.block<3,3>(10,10) = I3;        // accel bias
    m_F.block<3,3>(13,13) = I3;        // gyro bias
    m_F.block<4,3>(6,13)  = -G_w;      // gyro bias cross term

    m_Q.setZero();
    m_Q.block<6,6>(0,0)   = m_Q_trans;
    m_Q.block<4,4>(6,6)   = m_Q_att;
    m_Q.block<3,3>(10,10) = (SIGMA_BA_LOW * SIGMA_BA_LOW * dt) * I3;
    m_Q.block<3,3>(13,13) = (SIGMA_BG * SIGMA_BG * dt) * I3;

    // ── Propagate covariance ──────────────────────────────────────────────────
    m_P_temp.noalias() = m_F * m_P;
    m_P.noalias()      = m_P_temp * m_F.transpose() + m_Q;
    m_P_temp           = m_P + m_P.transpose();
    m_P                = 0.5f * m_P_temp;

}

void EKF::updateMag(const Eigen::Vector3f& z_meas_raw)
{
    using Mat3 = Eigen::Matrix3f;
    using Vec3 = Eigen::Vector3f;
    using Vec4 = Eigen::Vector4f;

    if (z_meas_raw.norm() < 1e-9f || m_mag_ref.norm() < 1e-9f) { return; }

    const Vec3 z_meas = z_meas_raw.normalized();    // data in body
    const Vec3 m_n    = m_mag_ref.normalized();     // ref in NED
    const float mN = m_n(0), mE = m_n(1), mD = m_n(2);

    Vec4 q = m_x.segment<4>(6);
    if (q.norm() < 1e-9f) { q = Vec4(1.0f, 0.0f, 0.0f, 0.0f); }
    else                  { q.normalize(); }
    const float q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);

    m_h.segment<3>(0) = Eigen::Quaternionf(q0, q1, q2, q3).toRotationMatrix().transpose() * m_n;

    Eigen::Matrix<float, 3, 4> Hq;
    Hq <<   -2*mD*q2 + 2*mE*q3,    2*mD*q3 + 2*mE*q2,              -2*mD*q0 + 2*mE*q1 - 4*mN*q2,    2*mD*q1 + 2*mE*q0 - 4*mN*q3,
             2*mD*q1 - 2*mN*q3,    2*mD*q0 - 4*mE*q1 + 2*mN*q2,     2*mD*q3 + 2*mN*q1,              2*mD*q2 - 4*mE*q3 - 2*mN*q0,   
            -2*mE*q1 + 2*mN*q2,   -4*mD*q1 - 2*mE*q0 + 2*mN*q3,    -4*mD*q2 + 2*mE*q3 + 2*mN*q0,    2*mE*q2 + 2*mN*q1;

    m_H.setZero();
    m_H.block<3,4>(0,6) = Hq;

    const Mat3 R = (SIGMA_MAG * SIGMA_MAG) * Mat3::Identity();
    m_y.segment<3>(0) = z_meas - m_h.segment<3>(0);
    const Mat3 S = m_H * m_P * m_H.transpose() + R;
    m_K = m_P * m_H.transpose() * S.ldlt().solve(Mat3::Identity());

    m_x += m_K * m_y.segment<3>(0);

    m_IKH.noalias()    = Eigen::Matrix<float,16,16>::Identity() - m_K * m_H;
    m_P_temp.noalias() = m_IKH * m_P;
    m_P.noalias()      = m_P_temp * m_IKH.transpose();
    m_P_temp.noalias() = m_K * R * m_K.transpose();
    m_P               += m_P_temp;
    m_P_temp           = m_P + m_P.transpose();
    m_P                = 0.5f * m_P_temp;

    Vec4 q_new = m_x.segment<4>(6);
    const float q_norm = q_new.norm();
    if (!std::isfinite(q_norm) || q_norm < 1e-9f)
        m_x.segment<4>(6) = Vec4(1.0f, 0.0f, 0.0f, 0.0f);
    else
    {
        q_new /= q_norm;
        if (q_new(0) < 0) q_new = -q_new;  // canonical hemisphere
        m_x.segment<4>(6) = q_new;
    }

}

void EKF::updateLowGAccel(const Eigen::Vector3f& z_accel)
{
    using Mat3 = Eigen::Matrix3f;
    using Vec3 = Eigen::Vector3f;
    using Vec4 = Eigen::Vector4f;

    const float z_norm = z_accel.norm();
    if (!std::isfinite(z_norm) || z_norm < 1e-9f) { return; }

    // Reject if not close to 1g → likely moving
    if (z_norm < 9.5f || z_norm > 10.1f) { return; }

    Vec4 q = m_x.segment<4>(6);
    if (q.norm() < 1e-9f) { q = Vec4(1.0f, 0.0f, 0.0f, 0.0f); }
    else                  { q.normalize(); }
    const float q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);

    const Vec3 ba_low = m_x.segment<3>(10);
    const Vec3 g_ned(0.0f, 0.0f, -g);
    m_h.segment<3>(3) = Eigen::Quaternionf(q0, q1, q2, q3).toRotationMatrix().transpose() * g_ned + ba_low;

    Eigen::Matrix<float, 3, 4> Hq; // derived assuming (0,0,-1)
    Hq <<    2*q2, -2*q3,   2*q0,  -2*q1,
            -2*q1, -2*q0,  -2*q3,  -2*q2,
                0,  4*q1,   4*q2,      0;
    Hq *= g;

    m_H.setZero();
    m_H.block<3,4>(0,6)  = Hq;
    m_H.block<3,3>(0,10) = Mat3::Identity();

    const Mat3 R = SIGMA_ACCEL_LOW.cwiseProduct(SIGMA_ACCEL_LOW).asDiagonal();
    m_y.segment<3>(3) = z_accel - m_h.segment<3>(3);
    const Mat3 S = m_H * m_P * m_H.transpose() + R;
    m_K = m_P * m_H.transpose() * S.ldlt().solve(Mat3::Identity());

    m_K.block<6,3>(0,0)  .setZero();   // position/vel

    m_x += m_K * m_y.segment<3>(3);

    m_IKH.noalias()    = Eigen::Matrix<float,16,16>::Identity() - m_K * m_H;
    m_P_temp.noalias() = m_IKH * m_P;
    m_P.noalias()      = m_P_temp * m_IKH.transpose();
    m_P_temp.noalias() = m_K * R * m_K.transpose();
    m_P               += m_P_temp;
    m_P_temp           = m_P + m_P.transpose();
    m_P                = 0.5f * m_P_temp;

    Vec4 q_new = m_x.segment<4>(6);
    const float q_norm = q_new.norm();
    if (!std::isfinite(q_norm) || q_norm < 1e-9f)
        m_x.segment<4>(6) = Vec4(1.0f, 0.0f, 0.0f, 0.0f);
    else
    {
        q_new /= q_norm;
        if (q_new(0) < 0) q_new = -q_new;  // canonical hemisphere
        m_x.segment<4>(6) = q_new;
    }
};

void EKF::updateBaro(const float pressure, const float temperature)
{
    using Mat2 = Eigen::Matrix<float, 2, 2>;
    using Vec2 = Eigen::Matrix<float, 2, 1>;

    // ── Use launch site as reference ──────────────────────────────────────────
    const float P_ref = m_setHome_ref.launch_pressure;
    const float T_ref = m_setHome_ref.launch_temperature;

    // ── Predicted altitude from state ─────────────────────────────────────────
    const float h      = -m_x(2);
    m_h(6) = T_ref + BARO_L * h;
    const float exp_   = (g * BARO_M_0) / (BARO_R_GAS * BARO_L);
    m_h(7) = P_ref * std::pow(T_ref / m_h(6), exp_);

    // ── Jacobians ─────────────────────────────────────────────────────────────
    const float dT_dh = BARO_L;
    const float dP_dh = -exp_ * BARO_L * m_h(7) / m_h(6);

    Eigen::Matrix<float, 2, 16> H_baro = Eigen::Matrix<float, 2, 16>::Zero();
    H_baro(0, 2) = -dT_dh;
    H_baro(1, 2) = -dP_dh;

    // ── Innovation ────────────────────────────────────────────────────────────
    Vec2 y;
    m_y(6) = temperature - m_h(6);
    m_y(7) = pressure - m_h(7);

    // ── Measurement noise ─────────────────────────────────────────────────────
    Mat2 R_baro = Mat2::Zero();
    R_baro(0,0) = SIGMA_T * SIGMA_T;
    R_baro(1,1) = SIGMA_P * SIGMA_P;

    // ── Kalman gain (16×2) ────────────────────────────────────────────────────
    const Mat2 S = H_baro * m_P * H_baro.transpose() + R_baro;
    const Eigen::Matrix<float, 16, 2> K_baro = m_P * H_baro.transpose() * S.ldlt().solve(Mat2::Identity());

    // ── State update ──────────────────────────────────────────────────────────
    m_x += K_baro * m_y.segment<2>(6);

    // ── Renormalise quaternion ────────────────────────────────────────────────
    Eigen::Vector4f q_new = m_x.segment<4>(6);
    const float q_norm = q_new.norm();
    if (!std::isfinite(q_norm) || q_norm < 1e-9f)
        m_x.segment<4>(6) = Eigen::Vector4f(1.0f, 0.0f, 0.0f, 0.0f);
    else
    {
        q_new /= q_norm;
        if (q_new(0) < 0) q_new = -q_new;
        m_x.segment<4>(6) = q_new;
    }

    // ── Joseph form covariance update ─────────────────────────────────────────
    m_IKH.noalias()    = Eigen::Matrix<float,16,16>::Identity() - K_baro * H_baro;
    m_P_temp.noalias() = m_IKH * m_P;
    m_P.noalias()      = m_P_temp * m_IKH.transpose();
    m_P_temp.noalias() = K_baro * R_baro * K_baro.transpose();
    m_P               += m_P_temp;
    m_P_temp           = m_P + m_P.transpose();
    m_P                = 0.5f * m_P_temp;
}

void EKF::updateGPS(const SensorStructs::GPS_t& gps)
{
    using Mat3 = Eigen::Matrix3f;
    using Mat6 = Eigen::Matrix<float, 6, 6>;
    using Vec3 = Eigen::Vector3f;
    using Vec6 = Eigen::Matrix<float, 6, 1>;

    // ── Quality gate ──────────────────────────────────────────────────────────
    if (!gps.valid || gps.fix < 1 || gps.sat < 4 || gps.hAcc > 3.0f) { return; }

    // ── Convert to radians in double ──────────────────────────────────────────
    const double lat0 = static_cast<double>(m_setHome_ref.launch_lat)  * 1e-7 * M_PI / 180.0;
    const double lon0 = static_cast<double>(m_setHome_ref.launch_lon)  * 1e-7 * M_PI / 180.0;
    const double lat  = static_cast<double>(gps.latitude)              * 1e-7 * M_PI / 180.0;
    const double lon  = static_cast<double>(gps.longitude)             * 1e-7 * M_PI / 180.0;
    const double h0   = static_cast<double>(m_setHome_ref.launch_alt);
    const double h    = static_cast<double>(gps.altitude);

    // ── LLA -> ECEF (measurement) ─────────────────────────────────────────────
    const double N  = GPS_A_EARTH / std::sqrt(1.0 - GPS_E2 * std::sin(lat) * std::sin(lat));
    const double X  = (N + h)               * std::cos(lat) * std::cos(lon);
    const double Y  = (N + h)               * std::cos(lat) * std::sin(lon);
    const double Z  = (N * (1.0 - GPS_E2) + h) * std::sin(lat);

    // ── LLA -> ECEF (reference) ───────────────────────────────────────────────
    const double N0 = GPS_A_EARTH / std::sqrt(1.0 - GPS_E2 * std::sin(lat0) * std::sin(lat0));
    const double X0 = (N0 + h0)                * std::cos(lat0) * std::cos(lon0);
    const double Y0 = (N0 + h0)                * std::cos(lat0) * std::sin(lon0);
    const double Z0 = (N0 * (1.0 - GPS_E2) + h0) * std::sin(lat0);

    // ── ECEF delta -> NED ─────────────────────────────────────────────────────
    const double dX = X - X0;
    const double dY = Y - Y0;
    const double dZ = Z - Z0;

    m_gps_position.setZero();
    m_gps_position(0) = static_cast<float>(-std::sin(lat0)*std::cos(lon0)*dX - std::sin(lat0)*std::sin(lon0)*dY + std::cos(lat0)*dZ);
    m_gps_position(1) = static_cast<float>(-std::sin(lon0)*dX               + std::cos(lon0)*dY);
    m_gps_position(2) = static_cast<float>(-std::cos(lat0)*std::cos(lon0)*dX - std::cos(lat0)*std::sin(lon0)*dY - std::sin(lat0)*dZ);

    // ── Velocity measurement (already in NED from GPS driver) ─────────────────
    const Vec3 z_vel(gps.v_n, gps.v_e, gps.v_d);

    // ── Combined measurement vector [pos; vel] ────────────────────────────────
    Vec6 z;
    z.segment<3>(0) = m_gps_position;
    z.segment<3>(3) = z_vel;

    // ── Measurement noise ─────────────────────────────────────────────────────
    const float sigma_ph = (gps.hAcc > 0.0f) ? gps.hAcc : 2.0f;
    const float sigma_pv = (gps.vAcc > 0.0f) ? gps.vAcc : 2.0f;

    Mat6 R_gps = Mat6::Zero();
    R_gps(0,0) = sigma_ph * sigma_ph;
    R_gps(1,1) = sigma_ph * sigma_ph;
    R_gps(2,2) = sigma_pv * sigma_pv;
    R_gps(3,3) = SIGMA_VEL * SIGMA_VEL;
    R_gps(4,4) = SIGMA_VEL * SIGMA_VEL;
    R_gps(5,5) = SIGMA_VEL * SIGMA_VEL;

    // ── Jacobian (6×16) ───────────────────────────────────────────────────────
    Eigen::Matrix<float, 6, 16> H_gps = Eigen::Matrix<float, 6, 16>::Zero();
    H_gps.block<3,3>(0,0) = Mat3::Identity();   // position
    H_gps.block<3,3>(3,3) = Mat3::Identity();   // velocity

    // ── Innovation ────────────────────────────────────────────────────────────
    m_h.segment<3>(8)  = m_x.segment<3>(0);   // predicted position
    m_h.segment<3>(11) = m_x.segment<3>(3);   // predicted velocity

    
    m_y.segment<3>(8)  = m_gps_position - m_h.segment<3>(8);   // position innovation
    m_y.segment<3>(11) = z_vel - m_h.segment<3>(11);  // velocity innovation

    const float speed = z_vel.norm();
    if (speed < 10.0f)
    {
        // stationary — only update velocity, skip position correction
        H_gps.block<3,3>(0,0).setZero();  // zero out position rows of H
        m_y.segment<3>(8).setZero();   // ← add this
    }

    const Mat6 S    = H_gps * m_P * H_gps.transpose() + R_gps;

    // ── Kalman gain (16×6) ────────────────────────────────────────────────────
    const Eigen::Matrix<float, 16, 6> K_gps = m_P * H_gps.transpose() * S.ldlt().solve(Mat6::Identity());

    // ── State update ──────────────────────────────────────────────────────────
    m_x += K_gps * m_y.segment<6>(8);

    // ── Renormalise quaternion ────────────────────────────────────────────────
    Eigen::Vector4f q_new = m_x.segment<4>(6);
    const float q_norm = q_new.norm();
    if (!std::isfinite(q_norm) || q_norm < 1e-9f)
        m_x.segment<4>(6) = Eigen::Vector4f(1.0f, 0.0f, 0.0f, 0.0f);
    else
    {
        q_new /= q_norm;
        if (q_new(0) < 0) q_new = -q_new;
        m_x.segment<4>(6) = q_new;
    }

    // ── Joseph form covariance update ─────────────────────────────────────────
    m_IKH.noalias()    = Eigen::Matrix<float,16,16>::Identity() - K_gps * H_gps;
    m_P_temp.noalias() = m_IKH * m_P;
    m_P.noalias()      = m_P_temp * m_IKH.transpose();
    m_P_temp.noalias() = K_gps * R_gps * K_gps.transpose();
    m_P               += m_P_temp;
    m_P_temp           = m_P + m_P.transpose();
    m_P                = 0.5f * m_P_temp;
}