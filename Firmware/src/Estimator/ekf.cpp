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
    m_x.segment<3>(13) = accel_bias;

    // Initial covariance — large uncertainty on everything except quaternion
    m_P.setZero();
    m_P.block<3,3>(0,0)   = 100.0f  * Eigen::Matrix3f::Identity();           // position
    m_P.block<3,3>(3,3)   = 10.0f   * Eigen::Matrix3f::Identity();           // velocity
    m_P.block<3,3>(6,6)   = 10.0f   * Eigen::Matrix3f::Identity();           // acceleration
    m_P.block<4,4>(9,9)   = 1.0f    * Eigen::Matrix<float,4,4>::Identity();  // quaternion
    m_P.block<3,3>(13,13) = 0.0f    * Eigen::Matrix3f::Identity();           // accel bias (bias frozen for now)

    m_gyro_bias = gyro_bias;
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

    predict(dt, gyro - m_gyro_bias);

    updateMag(mag);

    updateLowGAccel(accel);
}

void EKF::predict(const float dt, const Eigen::Vector3f gyro)
{
    using Mat3  = Eigen::Matrix3f;
    using Mat4  = Eigen::Matrix<float, 4, 4>;
    using Mat43 = Eigen::Matrix<float, 4, 3>;
    using Vec4  = Eigen::Vector4f;

    const Mat3 I3 = Mat3::Identity();

    // ── Extract state ─────────────────────────────────────────────────────────
    Vec4 q = m_x.segment<4>(9);
    q.normalize();
    const float q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);

    const float wx = gyro(0), wy = gyro(1), wz = gyro(2);

    // ── Attitude block ────────────────────────────────────────────────────────
    Mat4 Omega;
    Omega <<     0, -wx, -wy, -wz,
                wx,   0,  wz, -wy,
                wy, -wz,   0,  wx,
                wz,  wy, -wx,  0;

    Mat4 F_qq;
    const float w_norm = gyro.norm();
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

    // jacobian of prediction wrt angular rates
    Mat43 E_q;
    E_q << -q1, -q2, -q3,
            q0, -q3,  q2,
            q3,  q0, -q1,
           -q2,  q1,  q0;

    // ── Attitude process noise (quaternion only) ──────────────────
    const Eigen::Matrix<float, 4, 3> G_w = 0.5f * dt * E_q;
    m_Q_att = (SIGMA_ALPHA * SIGMA_ALPHA) * (G_w * G_w.transpose());

    // ── Accel bias process noise ──────────────────────────────────────────────
    const Mat3 Q_ba_low = (SIGMA_BA_LOW * SIGMA_BA_LOW * dt) * I3;

    // ── Full F and Q matrices (16×16) ─────────────────────────────────────────
    m_F.setZero();
    m_F.block<9,9>(0,0)   = m_F_trans;
    m_F.block<4,4>(9,9)   = F_qq;
    m_F.block<3,3>(13,13) = I3;   // accel bias — random walk

    m_Q.setZero();
    m_Q.block<9,9>(0,0)   = m_Q_trans;
    m_Q.block<4,4>(9,9)   = m_Q_att;
    m_Q.block<3,3>(13,13) = Q_ba_low;

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

    m_h.segment<3>(0) = Eigen::Quaternionf(q0, q1, q2, q3).toRotationMatrix().transpose() * m_n;

    Eigen::Matrix<float, 3, 4> Hq;
    Hq <<   -2*mD*q2 + 2*mE*q3,    2*mD*q3 + 2*mE*q2,              -2*mD*q0 + 2*mE*q1 - 4*mN*q2,    2*mD*q1 + 2*mE*q0 - 4*mN*q3,
             2*mD*q1 - 2*mN*q3,    2*mD*q0 - 4*mE*q1 + 2*mN*q2,     2*mD*q3 + 2*mN*q1,              2*mD*q2 - 4*mE*q3 - 2*mN*q0,   
            -2*mE*q1 + 2*mN*q2,   -4*mD*q1 - 2*mE*q0 + 2*mN*q3,    -4*mD*q2 + 2*mE*q3 + 2*mN*q0,    2*mE*q2 + 2*mN*q1;

    m_H.setZero();
    m_H.block<3,4>(0,9) = Hq;

    const Mat3 R = (SIGMA_MAG * SIGMA_MAG) * Mat3::Identity();
    m_y.segment<3>(0) = z_meas - m_h.segment<3>(0);
    const Mat3 S = m_H * m_P * m_H.transpose() + R;
    m_K = m_P * m_H.transpose() * S.ldlt().solve(Mat3::Identity());

    // m_x += m_K * m_y.segment<3>(0);

    m_IKH.noalias()    = Eigen::Matrix<float,16,16>::Identity() - m_K * m_H;
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
    {
        q_new /= q_norm;
        if (q_new(0) < 0) q_new = -q_new;  // canonical hemisphere
        m_x.segment<4>(9) = q_new;
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

    Vec4 q = m_x.segment<4>(9);
    if (q.norm() < 1e-9f) { q = Vec4(1.0f, 0.0f, 0.0f, 0.0f); }
    else                  { q.normalize(); }
    const float q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);

    const Vec3 ba_low = m_x.segment<3>(13);
    const Vec3 g_ned(0.0f, 0.0f, -g);
    m_h.segment<3>(3) = Eigen::Quaternionf(q0, q1, q2, q3).toRotationMatrix().transpose() * g_ned + ba_low;

    Eigen::Matrix<float, 3, 4> Hq; // derived assuming (0,0,-1)
    Hq <<    2*q2, -2*q3,   2*q0,  -2*q1,
            -2*q1, -2*q0,  -2*q3,  -2*q2,
                0,  4*q1,   4*q2,      0;
    Hq *= g;

    m_H.setZero();
    m_H.block<3,4>(0,9)  = Hq;
    m_H.block<3,3>(0,13) = Mat3::Identity();

    const Mat3 R = (SIGMA_ACCEL_LOW * SIGMA_ACCEL_LOW) * Mat3::Identity();
    m_y.segment<3>(3) = z_accel - m_h.segment<3>(3);
    const Mat3 S = m_H * m_P * m_H.transpose() + R;
    m_K = m_P * m_H.transpose() * S.ldlt().solve(Mat3::Identity());

    m_K.block<9,3>(0,0)  .setZero();   // position/vel/acc

    m_x += m_K * m_y.segment<3>(3);

    m_IKH.noalias()    = Eigen::Matrix<float,16,16>::Identity() - m_K * m_H;
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
    {
        q_new /= q_norm;
        if (q_new(0) < 0) q_new = -q_new;  // canonical hemisphere
        m_x.segment<4>(9) = q_new;
    }
}