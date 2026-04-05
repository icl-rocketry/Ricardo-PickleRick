#include "Estimator/ekf.h"

void EKF::setup(
    const Eigen::Vector3f& gyro_bias, 
    const Eigen::Vector3f& accel_bias, 
    const Eigen::Vector3f& mag_ref
)
{
    m_x.setZero();

    // Initialise quaternion to identity [1, 0, 0, 0]
    m_x(9) = 1.0f;

    // Seed bias states from calibration
    m_x.segment<3>(16) = gyro_bias;
    m_x.segment<3>(19) = accel_bias;

    // Initial covariance — large uncertainty on everything except quaternion
    m_P.setZero();
    m_P.block<3,3>(0,0)   = 100.0f  * Eigen::Matrix3f::Identity();           // position
    m_P.block<3,3>(3,3)   = 10.0f   * Eigen::Matrix3f::Identity();           // velocity
    m_P.block<3,3>(6,6)   = 10.0f   * Eigen::Matrix3f::Identity();           // acceleration
    m_P.block<4,4>(9,9)   = 1.0f    * Eigen::Matrix<float,4,4>::Identity();  // quaternion
    m_P.block<3,3>(13,13) = 0.0f    * Eigen::Matrix3f::Identity();           // angular rate (also frozen for now)
    m_P.block<3,3>(16,16) = 0.0f    * Eigen::Matrix3f::Identity();           // gyro bias  (bias frozen for now)
    m_P.block<3,3>(19,19) = 0.0f    * Eigen::Matrix3f::Identity();           // accel bias (bias frozen for now)

    m_mag_ref = mag_ref;
}

void EKF::update(const Eigen::Vector3f gyro, const Eigen::Vector3f accel, const Eigen::Vector3f mag)
{
    const uint32_t now = micros();  // use micros not millis for better dt resolution
    const float dt = (m_lastPredictTime == 0) 
                     ? 0.0f 
                     : static_cast<float>(now - m_lastPredictTime) * 1e-6f;
    m_lastPredictTime = now;

    if (dt <= 0.0f || dt > 0.5f) { return; }  // sanity check — skip bad dt

    predict(dt);

    // updateGyro(gyro);

    // updateMag(mag);

    updateLowGAccel(accel);
}

void EKF::predict(float dt)
{
    using Mat3  = Eigen::Matrix3f;
    using Mat4  = Eigen::Matrix<float, 4, 4>;
    using Mat43 = Eigen::Matrix<float, 4, 3>;
    using Mat73 = Eigen::Matrix<float, 7, 3>;
    using Vec3  = Eigen::Vector3f;
    using Vec4  = Eigen::Vector4f;

    const Mat3 I3 = Mat3::Identity();

    // ── Extract state ─────────────────────────────────────────────────────────
    Vec4 q = m_x.segment<4>(9);
    q.normalize();
    const float q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);

    const Vec3  w  = m_x.segment<3>(13);
    const float wx = w(0), wy = w(1), wz = w(2);

    // ── Translation block: position + velocity + acceleration (9×9) ──────────
    m_F_trans.setZero();
    m_F_trans.block<3,3>(0,0) = I3;
    m_F_trans.block<3,3>(0,3) = dt * I3;
    m_F_trans.block<3,3>(0,6) = 0.5f * dt * dt * I3;
    m_F_trans.block<3,3>(3,3) = I3;
    m_F_trans.block<3,3>(3,6) = dt * I3;
    m_F_trans.block<3,3>(6,6) = I3;

    // Translational process noise from constant-jerk model
    const float qj  = SIGMA_JERK * SIGMA_JERK;
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt3 * dt;
    const float dt5 = dt4 * dt;

    Eigen::Matrix3f Q_sub;
    Q_sub << dt5/20.0f,  dt4/8.0f,  dt3/6.0f,
             dt4/8.0f,   dt3/3.0f,  dt2/2.0f,
             dt3/6.0f,   dt2/2.0f,  dt;
    Q_sub *= qj;

    m_Q_trans.setZero();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            m_Q_trans.block<3,3>(i*3, j*3) = Q_sub(i,j) * I3;

    // ── Attitude block: quaternion + angular rate (7×7) ───────────────────────
    Mat4 Omega;
    Omega <<  0.0f, -wx,  -wy,  -wz,
              wx,   0.0f,  wz,  -wy,
              wy,  -wz,   0.0f,  wx,
              wz,   wy,  -wx,   0.0f;

    Mat4 F_qq;
    const float w_norm = w.norm();
    if (w_norm > 1e-9f)
    {
        F_qq = std::cos(0.5f * w_norm * dt) * Mat4::Identity()
             + (std::sin(0.5f * w_norm * dt) / w_norm) * 0.5f * Omega;
    }
    else
    {
        F_qq = Mat4::Identity() + 0.5f * dt * Omega;
    }

    Mat43 E_q;
    E_q << -q1, -q2, -q3,
            q0, -q3,  q2,
            q3,  q0, -q1,
           -q2,  q1,  q0;

    const Mat43 F_qw = 0.5f * dt * E_q;

    m_F_att.setZero();
    m_F_att.block<4,4>(0,0) = F_qq;
    m_F_att.block<4,3>(0,4) = F_qw;
    m_F_att.block<3,3>(4,4) = I3;

    Mat73 G_w;
    G_w.block<4,3>(0,0) = 0.5f * dt * E_q;
    G_w.block<3,3>(4,0) = I3;

    m_Q_att = (SIGMA_ALPHA * SIGMA_ALPHA * dt) * (G_w * G_w.transpose());

    // ── Bias process noise ────────────────────────────────────────────────────
    const Mat3 Q_bg     = (SIGMA_BG     * SIGMA_BG     * dt) * I3;
    const Mat3 Q_ba_low = (SIGMA_BA_LOW * SIGMA_BA_LOW * dt) * I3;

    // ── Full F and Q matrices (22×22) ─────────────────────────────────────────
    m_F.setZero();
    m_F.block<9,9>(0,0)   = m_F_trans;
    m_F.block<7,7>(9,9)   = m_F_att;
    m_F.block<3,3>(16,16) = I3;
    m_F.block<3,3>(19,19) = I3;

    m_Q.setZero();
    m_Q.block<9,9>(0,0)   = m_Q_trans;
    m_Q.block<7,7>(9,9)   = m_Q_att;
    m_Q.block<3,3>(16,16) = Q_bg;
    m_Q.block<3,3>(19,19) = Q_ba_low;

    // ── Propagate state ───────────────────────────────────────────────────────
    m_x.segment<9>(0) = m_F_trans * m_x.segment<9>(0);
    m_x.segment<4>(9) = F_qq * q;
    m_x.segment<4>(9).normalize();

    // ── Propagate covariance ──────────────────────────────────────────────────
    m_P_temp.noalias() = m_F * m_P;
    m_P.noalias()      = m_P_temp * m_F.transpose() + m_Q;
    m_P_temp           = m_P + m_P.transpose();
    m_P                = 0.5f * m_P_temp;
}

void EKF::updateGyro(const Eigen::Vector3f& z_gyro)
{
    using Mat3 = Eigen::Matrix3f;

    if (!z_gyro.allFinite()) { return; }

    const Mat3 R = (SIGMA_GYRO * SIGMA_GYRO) * Mat3::Identity();

    const Eigen::Vector3f h = m_x.segment<3>(16);

    m_H.setZero();
    m_H.block<3,3>(0,16) = Mat3::Identity();

    const Eigen::Vector3f y = z_gyro - h;
    const Mat3 S = m_H * m_P * m_H.transpose() + R;
    m_K = m_P * m_H.transpose() * S.ldlt().solve(Mat3::Identity());

    // ── Freeze bias — only angular rate (13-15) receives correction ───────────
    m_K.block<3,3>(16,0).setZero();   // stop gyro bias updating (for now)


    m_x += m_K * y;

    m_IKH.noalias()    = Eigen::Matrix<float,22,22>::Identity() - m_K * m_H;
    m_P_temp.noalias() = m_IKH * m_P;
    m_P.noalias()      = m_P_temp * m_IKH.transpose();
    m_P_temp.noalias() = m_K * R * m_K.transpose();
    m_P               += m_P_temp;
    m_P_temp           = m_P + m_P.transpose();
    m_P                = 0.5f * m_P_temp;

    Eigen::Vector4f q = m_x.segment<4>(9);
    const float q_norm = q.norm();
    if (!std::isfinite(q_norm) || q_norm < 1e-9f)
        m_x.segment<4>(9) = Eigen::Vector4f(1.0f, 0.0f, 0.0f, 0.0f);
    else
        m_x.segment<4>(9) = q / q_norm;

    char buf[120];
    snprintf(buf, sizeof(buf),
        "gyro z=[%.3f,%.3f,%.3f] h=[%.3f,%.3f,%.3f] y=[%.3f,%.3f,%.3f]",
        z_gyro(0), z_gyro(1), z_gyro(2),
        h(0), h(1), h(2),
        y(0), y(1), y(2));
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(buf);
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

    Vec4 q = m_x.segment<4>(9);
    if (q.norm() < 1e-9f) { q = Vec4(1.0f, 0.0f, 0.0f, 0.0f); }
    else                  { q.normalize(); }
    const float q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);

    const Vec3 h = Eigen::Quaternionf(q0, q1, q2, q3).toRotationMatrix() * m_n;

    Eigen::Matrix<float, 3, 4> Hq;
    Hq << 2*q3*mE - 2*q2*mD,         2*q2*mE + 2*q3*mD,         -4*q2*mN + 2*q1*mE - 2*q0*mD, -4*q3*mN + 2*q0*mE + 2*q1*mD,
         -2*q3*mN + 2*q1*mD,         2*q2*mN - 4*q1*mE + 2*q0*mD, 2*q1*mN + 2*q3*mD,           -2*q0*mN - 4*q3*mE + 2*q2*mD,
          2*q2*mN - 2*q1*mE,         2*q3*mN - 2*q0*mE - 4*q1*mD, 2*q0*mN + 2*q3*mE - 4*q2*mD,  2*q1*mN + 2*q2*mE;
    // Hq = Hq * (Mat4::Identity() - q * q.transpose());

    m_H.setZero();
    m_H.block<3,4>(0,9) = - Hq;

    const Mat3 R = (SIGMA_MAG * SIGMA_MAG) * Mat3::Identity();
    const Vec3 y = z_meas - h;
    const Mat3 S = m_H * m_P * m_H.transpose() + R;
    m_K = m_P * m_H.transpose() * S.ldlt().solve(Mat3::Identity());

    m_x += m_K * y;

    m_IKH.noalias()    = Eigen::Matrix<float,22,22>::Identity() - m_K * m_H;
    m_P_temp.noalias() = m_IKH * m_P;
    m_P.noalias()      = m_P_temp * m_IKH.transpose();
    m_P_temp.noalias() = m_K * R * m_K.transpose();
    m_P               += m_P_temp;
    m_P_temp           = m_P + m_P.transpose();
    m_P                = 0.5f * m_P_temp;

    Vec4 q_new = m_x.segment<4>(9);
    const float q_norm = q_new.norm();
    if (!std::isfinite(q_norm) || q_norm < 1e-9f)
        m_x.segment<4>(9) = Vec4(1.0f, 0.0f, 0.0f, 0.0f);
    else
        m_x.segment<4>(9) = q_new / q_norm;

    char buf[120];
    snprintf(buf, sizeof(buf),
        "mag z=[%.3f,%.3f,%.3f] h=[%.3f,%.3f,%.3f] y=[%.3f,%.3f,%.3f]",
        z_meas(0), z_meas(1), z_meas(2),
        h(0), h(1), h(2),
        y(0), y(1), y(2));
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(buf);
}

void EKF::updateLowGAccel(const Eigen::Vector3f& z_accel)
{
    using Mat3 = Eigen::Matrix3f;
    using Vec3 = Eigen::Vector3f;
    using Vec4 = Eigen::Vector4f;

    const float z_norm = z_accel.norm();
    if (!std::isfinite(z_norm) || z_norm < 1e-9f) { return; }

    if (std::abs(z_norm - g) > 2.0f) { return; }

    Vec4 q = m_x.segment<4>(9);
    if (q.norm() < 1e-9f) { q = Vec4(1.0f, 0.0f, 0.0f, 0.0f); }
    else                  { q.normalize(); }
    const float q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);

    const Vec3 ba_low = m_x.segment<3>(19);
    const Vec3 g_ned(0.0f, 0.0f, -g);
    const Vec3 h = Eigen::Quaternionf(q0, q1, q2, q3).toRotationMatrix() * g_ned + ba_low;

    Eigen::Matrix<float, 3, 4> Hq;
    Hq <<  2*q2,  -2*q3,   2*q0,  -2*q1,
          -2*q1,  -2*q0,  -2*q3,  -2*q2,
          -2*q0,   2*q1,   2*q2,  -2*q3;
    Hq *= -g;

    m_H.setZero();
    m_H.block<3,4>(0,9)  = Hq;
    m_H.block<3,3>(0,19) = Mat3::Identity();

    const Mat3 R = (SIGMA_ACCEL_LOW * SIGMA_ACCEL_LOW) * Mat3::Identity();
    const Vec3 y = z_accel - h;
    const Mat3 S = m_H * m_P * m_H.transpose() + R;
    m_K = m_P * m_H.transpose() * S.ldlt().solve(Mat3::Identity());

    // Only quaternion (9-12) and ba_low (19-21) receive corrections
    m_K.block<9,3>(0,0)  .setZero();   // position/vel/acc
    m_K.block<3,3>(13,0) .setZero();   // angular rate
    m_K.block<3,3>(16,0) .setZero();   // gyro bias

    m_x += m_K * y;

    m_IKH.noalias()    = Eigen::Matrix<float,22,22>::Identity() - m_K * m_H;
    m_P_temp.noalias() = m_IKH * m_P;
    m_P.noalias()      = m_P_temp * m_IKH.transpose();
    m_P_temp.noalias() = m_K * R * m_K.transpose();
    m_P               += m_P_temp;
    m_P_temp           = m_P + m_P.transpose();
    m_P                = 0.5f * m_P_temp;

    Vec4 q_new = m_x.segment<4>(9);
    const float q_norm = q_new.norm();
    if (!std::isfinite(q_norm) || q_norm < 1e-9f)
        m_x.segment<4>(9) = Vec4(1.0f, 0.0f, 0.0f, 0.0f);
    else
        m_x.segment<4>(9) = q_new / q_norm;

    char buf[120];
    snprintf(buf, sizeof(buf),
        "accel z=[%.3f,%.3f,%.3f] h=[%.3f,%.3f,%.3f] y=[%.3f,%.3f,%.3f]",
        z_accel(0), z_accel(1), z_accel(2),
        h(0), h(1), h(2),
        y(0), y(1), y(2));
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(buf);

}