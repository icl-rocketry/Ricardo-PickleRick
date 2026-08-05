#include "Estimator/ekf.h"
#include "Config/debug_config.h"
#include "Config/timing_config.h"
#include <cmath>
#include <limits>

namespace
{
    bool timeAtOrAfter(const uint32_t lhs, const uint32_t rhs)
    {
        return static_cast<int32_t>(lhs - rhs) >= 0;
    }

    bool timeAfter(const uint32_t lhs, const uint32_t rhs)
    {
        return static_cast<int32_t>(lhs - rhs) > 0;
    }

    bool isNewMeasurement(const uint32_t timestamp_us, const uint32_t last_timestamp_us)
    {
        return timestamp_us != 0 &&
               (last_timestamp_us == 0 || timeAfter(timestamp_us, last_timestamp_us));
    }

    bool gnssEpochAfter(const uint32_t epoch_ms, const uint32_t previous_epoch_ms)
    {
        constexpr int32_t day_ms = 86400000;
        constexpr int32_t half_day_ms = day_ms / 2;
        int32_t delta_ms = static_cast<int32_t>(epoch_ms) -
                           static_cast<int32_t>(previous_epoch_ms);
        if (delta_ms < -half_day_ms) { delta_ms += day_ms; }
        if (delta_ms >  half_day_ms) { delta_ms -= day_ms; }
        return delta_ms > 0;
    }

    // 99% upper-tail chi-square thresholds. The active measurement dimension,
    // rather than the storage-vector size, determines the appropriate gate.
    float nisGateThreshold(const uint8_t degrees_of_freedom)
    {
        switch (degrees_of_freedom)
        {
            case 1: return 6.63490f;
            case 2: return 9.21034f;
            case 3: return 11.34487f;
            case 4: return 13.27670f;
            case 5: return 15.08627f;
            default: return 16.81189f; // 6 DoF is the largest measurement used here.
        }
    }

    template <typename Decomposition, typename Vector>
    bool calculateNis(const Decomposition& decomposition,
                      const Vector& innovation,
                      float& nis)
    {
        if (!innovation.allFinite() ||
            decomposition.info() != Eigen::Success ||
            !decomposition.isPositive())
        {
            nis = std::numeric_limits<float>::quiet_NaN();
            return false;
        }

        const Vector normalized_innovation = decomposition.solve(innovation);
        nis = innovation.dot(normalized_innovation);
        return decomposition.info() == Eigen::Success &&
               normalized_innovation.allFinite() &&
               std::isfinite(nis) &&
               nis >= 0.0f;
    }

    bool timerDue(const uint32_t now, uint32_t& previous, const uint32_t period) //decides whether an update should be made
    {
        if (previous == 0)
        {
            previous = now;
            return false;
        }

        if (now - previous < period)
        {
            return false;
        }

        previous += period;
        if (now - previous >= period)
        {
            previous = now;
        }

        return true;
    }

    bool correctionDueNow(const uint32_t now, uint32_t& previous, const uint32_t period)
    {
        if (previous == 0 || now - previous >= period)
        {
            previous = now;
            return true;
        }

        return false;
    }

    bool isFiniteVector(const Eigen::Vector3f& v)
    {
        return std::isfinite(v.x()) &&
               std::isfinite(v.y()) &&
               std::isfinite(v.z());
    }
}

void EKF::setup(const Eigen::Vector3f& gyro_bias, 
                const Eigen::Vector3f& accel_bias, 
                const Eigen::Vector3f& h_accel_bias,
                const Eigen::Vector3f& mag_ref
)
{
    m_x.setZero();

    // Fallback attitude until accel+mag startup alignment has settled.
    m_x(6) = 0.70710678f;  // q0
    m_x(7) = 0.0f;         // q1
    m_x(8) = 0.70710678f;  // q2
    m_x(9) = 0.0f;         // q3

    // Seed bias states from calibration
    m_x.segment<3>(10) = accel_bias;
    m_x.segment<3>(13) = gyro_bias;
    m_h_accel_bias  = h_accel_bias;
    m_mag_ref       = mag_ref;
    m_lastPredictTime = 0;
    m_lastCovarianceUpdateTime = 0;
    m_lastAccelCorrectionTime = 0;
    m_lastMagCorrectionTime = 0;
    m_lastBaroCorrectionTime = 0;
    m_lastGpsCorrectionTime = 0;
    m_lastLidarCorrectionTime = 0;
    m_lastRtkCorrectionTime = 0;
    m_lastMagMeasurementTime = 0;
    m_lastBaroMeasurementTime = 0;
    m_lastGpsMeasurementTime = 0;
    m_lastLidarMeasurementTime = 0;
    m_lastRtkMeasurementTime = 0;
    m_lastRtkMeasurementEpochMs = 0;
    m_lastHandledGpsTimestampUs = 0;
    m_lastFusedGpsTimestampUs = 0;
    m_lastHandledRtkTimestampUs = 0;
    m_lastHandledRtkEpochMs = 0;
    m_lastFusedRtkTimestampUs = 0;
    m_lastAttitudeInitMagMeasurementTime = 0;
    m_covariancePredictDt = 0.0f;
    m_nextCorrectionIndex = 0;
    m_gnssTimeOffsetValid = false;
    m_gnssToLocalOffsetUs = 0;
    m_lastRtkDelayUs = 0;
    m_attitudeInitialised = false;
    m_attitudeInitSampleCount = 0;
    m_attitudeInitAccelAccum.setZero();
    m_attitudeInitMagAccum.setZero();
    m_acceleration.setZero();
    m_angular_rates.setZero();
    m_gps_position.setZero();
    m_h.setZero();
    m_y.setZero();
    m_magNis = -1.0f;
    m_accelNis = -1.0f;
    m_baroNis = -1.0f;
    m_gpsNis = -1.0f;
    m_rtkNis = -1.0f;
    m_lidarNis = -1.0f;
    m_magNisCount = 0;
    m_accelNisCount = 0;
    m_baroNisCount = 0;
    m_gpsNisCount = 0;
    m_rtkNisCount = 0;
    m_lidarNisCount = 0;
    m_magNisTimestampUs = 0;
    m_accelNisTimestampUs = 0;
    m_baroNisTimestampUs = 0;
    m_gpsNisTimestampUs = 0;
    m_rtkNisTimestampUs = 0;
    m_lidarNisTimestampUs = 0;
    m_magNisRejectReason = SensorStructs::NisRejectReason::NOT_CALCULATED_YET;
    m_accelNisRejectReason = SensorStructs::NisRejectReason::NOT_CALCULATED_YET;
    m_baroNisRejectReason = SensorStructs::NisRejectReason::NOT_CALCULATED_YET;
    m_gpsNisRejectReason = SensorStructs::NisRejectReason::NOT_CALCULATED_YET;
    m_rtkNisRejectReason = SensorStructs::NisRejectReason::NOT_CALCULATED_YET;
    m_lidarNisRejectReason = SensorStructs::NisRejectReason::NOT_CALCULATED_YET;
    m_magNisRejectTimestampUs = 0;
    m_accelNisRejectTimestampUs = 0;
    m_baroNisRejectTimestampUs = 0;
    m_gpsNisRejectTimestampUs = 0;
    m_rtkNisRejectTimestampUs = 0;
    m_lidarNisRejectTimestampUs = 0;
    m_lastGpsNisMeasurementTime = 0;
    m_lastAccelNisMeasurementTime = 0;
    m_lastMagNisMeasurementTime = 0;
    m_lastBaroNisMeasurementTime = 0;
    m_lastRtkNisMeasurementTime = 0;
    m_lastLidarNisMeasurementTime = 0;
    if constexpr (!TimingConfig::EKF::GPS_CORRECTION_ENABLED)
    {
        m_gpsNisRejectReason = SensorStructs::NisRejectReason::NIS_DISABLED;
    }
    if constexpr (!TimingConfig::EKF::LIDAR_CORRECTION_ENABLED)
    {
        m_lidarNisRejectReason = SensorStructs::NisRejectReason::NIS_DISABLED;
    }
    resetHistory();

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

void EKF::update(   const Eigen::Vector3f         gyro,
                    const Eigen::Vector3f         accel,
                    const Eigen::Vector3f         h_accel,
                    const SensorStructs::MAG_3AXIS_t& mag,
                    const SensorStructs::BARO_t&  baro,
                    const SensorStructs::GPS_t&   gps,
                    const SensorStructs::LIDAR_t& lidar,
                    const SensorStructs::RTK_t&   rtk
                )
{
    const uint32_t now = micros();  // use micros not millis for better dt resolution
    updateGnssTimeOffset(gps);

    if (!m_attitudeInitialised)
    {
        m_acceleration = accel - m_x.segment<3>(10);
        m_angular_rates = gyro - m_x.segment<3>(13);

        if (initialiseAttitudeIfSettled(gyro, accel, mag))
        {
            m_lastPredictTime = now;
            m_covariancePredictDt = 0.0f;
            resetHistory();
        }
        return;
    }

    const float dt = (m_lastPredictTime == 0) 
    ? 0.0f 
    : static_cast<float>(now - m_lastPredictTime) * 1e-6f;
    m_lastPredictTime = now;
    
    if (dt <= 0.0f || dt > 0.5f) { return; }  // sanity check — skip bad dt

    m_covariancePredictDt += dt;
    const bool propagate_covariance = timerDue(now, m_lastCovarianceUpdateTime, TimingConfig::EKF::COVARIANCE_UPDATE_DELTA_US);
    const float covariance_dt = propagate_covariance ? m_covariancePredictDt : dt;

    if (propagate_covariance)
    {
        m_covariancePredictDt = 0.0f;
    }

    predict(dt, covariance_dt, propagate_covariance, gyro, accel, h_accel);
    saveHistorySample(now,
                      dt,
                      covariance_dt,
                      propagate_covariance,
                      gyro,
                      accel,
                      h_accel,
                      mag,
                      baro,
                      gps,
                      lidar,
                      rtk);

    bool gps_replayed_to_now = false;
    if constexpr (TimingConfig::EKF::GPS_CORRECTION_ENABLED)
    {
        gps_replayed_to_now = handleGpsCorrection(now, gps); // Fuse fresh GPS, replaying delayed data when possible.
    }
    const bool rtk_replayed_to_now = gps_replayed_to_now ? false : handleRtkCorrection(now, rtk);
    if (!gps_replayed_to_now && !rtk_replayed_to_now)
    {
        // GPS and RTK enter through their delayed-measurement handlers above.
        // The normal scheduler only services the remaining sensors.
        runScheduledCorrection(now, accel, mag, baro, gps, lidar, rtk, false, false);
    }

    overwriteLatestHistoryState();
}

void EKF::setHome(const SensorStructs::home_ref_t& setHome_ref) 
{ 
    m_setHome_ref = setHome_ref; 
    m_x.segment<6>(0).setZero();
};

void EKF::predict(  const float nominal_dt,
                    const float covariance_dt,
                    const bool propagate_covariance,
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
    // Angular rates in body frame, corrected for gyro bias
    const float wx = gyro(0) - m_x(13);
    const float wy = gyro(1) - m_x(14);
    const float wz = gyro(2) - m_x(15);
    m_angular_rates << wx, wy, wz;
    // Angular-rate norm used to normalise Omega; near zero we avoid dividing by it.
    const float w_norm = m_angular_rates.norm();

    Mat4 Omega;
    Omega <<     0, -wx, -wy, -wz,
                wx,   0,  wz, -wy,
                wy, -wz,   0,  wx,
                wz,  wy, -wx,   0;

    Mat4 F_qq; // generate the quaternion update matrix for the state transition
    if (w_norm > 1e-9f)
    {
        // exp((dt/2)*omega) [dont need i from e^ix = cosx + isinx for weird maths reasons]
        F_qq =  std::cos(0.5f * w_norm * nominal_dt) * Mat4::Identity()
              + std::sin(0.5f * w_norm * nominal_dt) * (Omega / w_norm);
    }
    else // to avoid the /0, use first-order Taylor expansion of sin(x)/x ~ 1 for small x
    {
        F_qq = Mat4::Identity() + 0.5f * nominal_dt * Omega;
    }
        
    m_x.segment<4>(6) = F_qq * q; // update quaternion state
    m_x.segment<4>(6).normalize();

    // ── Translation update ───────────────────────────────────────────────────────

    // ── Select accelerometer
    const float accel_norm   = accel.norm();

    if (accel_norm < LOW_G_SATURATION)          // low-g not saturated
    {
        m_acceleration = accel - m_x.segment<3>(10); //take away bias
    }
    else
    {
        m_acceleration = h_accel - m_h_accel_bias;
    }

    Vec4 q_new = m_x.segment<4>(6);
    const Mat3 R_body_to_ned = Eigen::Quaternionf(q_new(0), q_new(1), q_new(2), q_new(3)).toRotationMatrix();

    const Eigen::Vector3f g_ned(0.0f, 0.0f, -g);

    const Eigen::Vector3f a_ned = R_body_to_ned * (m_acceleration) - g_ned; 
    const float nominal_dt2 = nominal_dt * nominal_dt;

    if (USE_ACCEL_FOR_VELOCITY) {
        m_x.segment<3>(0) += m_x.segment<3>(3) * nominal_dt + 0.5f * a_ned * nominal_dt2;
        m_x.segment<3>(3) += a_ned * nominal_dt;
    } else {
        m_x.segment<3>(0) += m_x.segment<3>(3) * nominal_dt;
    }

    if (!propagate_covariance)
    {
        return;
    }

    const float dt = covariance_dt;
    const float dt2 = dt * dt;

    Mat4 F_qq_cov;
    if (w_norm > 1e-9f)
    {
        F_qq_cov =  std::cos(0.5f * w_norm * dt) * Mat4::Identity()
                 + std::sin(0.5f * w_norm * dt) * (Omega / w_norm);
    }
    else
    {
        F_qq_cov = Mat4::Identity() + 0.5f * dt * Omega;
    }

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
           

    // ── Translation process noise ────────────────────────────────────────────────

    const float dt3 = dt2 * dt;
    const float qa  = SIGMA_ACCEL_PROCESS * SIGMA_ACCEL_PROCESS;// units m^2/s^3, which is the variance of the acceleration noise per unit time
    Eigen::Matrix<float, 2, 2> Q_sub;
    Q_sub << qa * dt3 / 3.0f,  qa * dt2 / 2.0f,
             qa * dt2 / 2.0f,  qa * dt;

    // Q_trans = kron(Q_sub, I3) — 6×6
    m_Q_trans.setZero();
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
            m_Q_trans.block<3,3>(i*3, j*3) = Q_sub(i,j) * I3;


    // ── Full F and Q matrices (16×16) ─────────────────────────────────────────
    // F is the Jacobian of the process model 
    m_F.setZero();
    m_F.block<3,3>(0,0)   = I3;            // position integrates
    m_F.block<3,3>(0,3)   = dt * I3;       // position depends on velocity
    m_F.block<3,3>(3,3)   = I3;            // velocity integrates
    m_F.block<4,4>(6,6)   = F_qq_cov;      // attitude
    m_F.block<3,3>(10,10) = I3;            // accel bias
    m_F.block<3,3>(13,13) = I3;            // gyro bias
    m_F.block<4,3>(6,13)  = -G_w;          // gyro bias cross term

    const float qw = q_new(0), qx = q_new(1), qy = q_new(2), qz = q_new(3);
    const float ax = m_acceleration(0), ay = m_acceleration(1), az = m_acceleration(2);
    Eigen::Matrix<float, 3, 4> d_accel_ned_dq;
    d_accel_ned_dq << -2.0f*qz*ay + 2.0f*qy*az, // this shows how each state is affected by an error in the quaternion and vice versa. 
                       2.0f*qy*ay + 2.0f*qz*az, // The Jacobian is used to propagate the uncertainty in the quaternion to the uncertainty in the acceleration in NED frame.
                      -4.0f*qy*ax + 2.0f*qx*ay + 2.0f*qw*az,
                      -4.0f*qz*ax - 2.0f*qw*ay + 2.0f*qx*az,

                       2.0f*qz*ax - 2.0f*qx*az,
                       2.0f*qy*ax - 4.0f*qx*ay - 2.0f*qw*az,
                       2.0f*qx*ax + 2.0f*qz*az,
                       2.0f*qw*ax - 4.0f*qz*ay + 2.0f*qy*az,

                      -2.0f*qy*ax + 2.0f*qx*ay,
                       2.0f*qz*ax + 2.0f*qw*ay - 4.0f*qx*az,
                      -2.0f*qw*ax + 2.0f*qz*ay - 4.0f*qy*az,
                       2.0f*qx*ax + 2.0f*qy*ay;

    m_F.block<3,4>(3,6)   = d_accel_ned_dq * dt; //velocity depends on acceleration and attitude
    m_F.block<3,4>(0,6)   = 0.5f * d_accel_ned_dq * dt2; // position depends on acceleration and attitude
    m_F.block<3,3>(3,10)  = -R_body_to_ned * dt; //accelerometer bias a_ned = R * (accel - accel_bias) - g, d a_ned / d accel_bias = -R
    m_F.block<3,3>(0,10)  = -0.5f * R_body_to_ned * dt2; // accelerometer bias d v_next / d accel_bias = -R * dt
    // Q is the process noise covariance — how much we trust the process model (vs measurements)
    m_Q.setZero();
    m_Q.block<6,6>(0,0)   = m_Q_trans;
    m_Q.block<4,4>(6,6)   = m_Q_att;
    m_Q.block<3,3>(10,10) = (SIGMA_BA_LOW.array().square().matrix().asDiagonal()) * dt;
    m_Q.block<3,3>(13,13) = (SIGMA_BG.array().square().matrix().asDiagonal()) * dt;

    // ── Propagate covariance ──────────────────────────────────────────────────
    m_P_temp.noalias() = m_F * m_P;
    m_P.noalias()      = m_P_temp * m_F.transpose() + m_Q;
    m_P_temp           = m_P + m_P.transpose(); // this line is to deal with floating point round off errors 
    m_P                = 0.5f * m_P_temp;

}

void EKF::updateMag(const Eigen::Vector3f& z_meas_raw,
                    const uint32_t measurement_time_us)
{
    using Vec3 = Eigen::Vector3f;
    using Vec4 = Eigen::Vector4f;

    const bool publish_nis = isNewMeasurement(measurement_time_us,
                                               m_lastMagNisMeasurementTime);
    if (publish_nis)
    {
        m_lastMagNisMeasurementTime = measurement_time_us;
    }

    if (!isFiniteVector(z_meas_raw) || z_meas_raw.norm() < 1e-9f)
    {
        if (publish_nis)
        {
            m_magNisRejectReason = SensorStructs::NisRejectReason::INVALID_MEASUREMENT;
            m_magNisRejectTimestampUs = micros();
        }
        return;
    }
    if (!isFiniteVector(m_mag_ref) || m_mag_ref.norm() < 1e-9f)
    {
        if (publish_nis)
        {
            m_magNisRejectReason = SensorStructs::NisRejectReason::INVALID_REFERENCE;
            m_magNisRejectTimestampUs = micros();
        }
        return;
    }

    const Vec3 z_meas = z_meas_raw.normalized();    // data in body
    const Vec3 m_n    = m_mag_ref.normalized();     // ref in NED

    Vec4 q = m_x.segment<4>(6);
    if (q.norm() < 1e-9f) { q = Vec4(1.0f, 0.0f, 0.0f, 0.0f); }
    else                  { q.normalize(); }
    const float q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);

    Eigen::Quaternionf q_body_to_ned(q0, q1, q2, q3);
    const Eigen::Matrix3f R_body_to_ned = q_body_to_ned.toRotationMatrix();

    // Keep full-field diagnostics, but only fuse the horizontal heading error.
    m_h.segment<3>(0) = R_body_to_ned.transpose() * m_n;
    m_y.segment<3>(0) = z_meas - m_h.segment<3>(0);

    Vec3 measured_horizontal_ned = R_body_to_ned * z_meas;
    measured_horizontal_ned.z() = 0.0f;
    const float measured_horizontal_norm = measured_horizontal_ned.norm();
    if (!std::isfinite(measured_horizontal_norm) || measured_horizontal_norm < 1e-6f)
    {
        if (publish_nis)
        {
            m_magNisRejectReason = SensorStructs::NisRejectReason::INVALID_MEASUREMENT;
            m_magNisRejectTimestampUs = micros();
        }
        return;
    }
    measured_horizontal_ned /= measured_horizontal_norm;

    Vec3 reference_horizontal_ned = m_n;
    reference_horizontal_ned.z() = 0.0f;
    const float reference_horizontal_norm = reference_horizontal_ned.norm();
    if (!std::isfinite(reference_horizontal_norm) || reference_horizontal_norm < 1e-6f)
    {
        if (publish_nis)
        {
            m_magNisRejectReason = SensorStructs::NisRejectReason::INVALID_REFERENCE;
            m_magNisRejectTimestampUs = micros();
        }
        return;
    }
    reference_horizontal_ned /= reference_horizontal_norm;

    float dot = measured_horizontal_ned.x() * reference_horizontal_ned.x()
              + measured_horizontal_ned.y() * reference_horizontal_ned.y();
    if (dot > 1.0f)  { dot = 1.0f; }
    if (dot < -1.0f) { dot = -1.0f; }

    const float cross_down = measured_horizontal_ned.x() * reference_horizontal_ned.y()
                           - measured_horizontal_ned.y() * reference_horizontal_ned.x();
    const float heading_error = std::atan2(cross_down, dot);

    Eigen::Matrix<float, 1, 16> H_heading = Eigen::Matrix<float, 1, 16>::Zero();
    const Eigen::Matrix<float, 4, 1> yaw_tangent(
        -0.5f * q3,
        -0.5f * q2,
         0.5f * q1,
         0.5f * q0
    );
    const float yaw_tangent_norm_sq = yaw_tangent.squaredNorm();
    if (!std::isfinite(yaw_tangent_norm_sq) || yaw_tangent_norm_sq < 1e-9f)
    {
        if (publish_nis)
        {
            m_magNisRejectReason = SensorStructs::NisRejectReason::NUMERIC_FAILURE;
            m_magNisRejectTimestampUs = micros();
        }
        return;
    }

    H_heading.block<1,4>(0,6) = yaw_tangent.transpose() / yaw_tangent_norm_sq;

    const float R_heading = SIGMA_MAG_HEADING * SIGMA_MAG_HEADING;
    float heading_variance = (H_heading * m_P * H_heading.transpose())(0, 0);
    if (!std::isfinite(heading_variance) || heading_variance < 0.0f) {
        heading_variance = 0.0f;
    }

    const float S = heading_variance + R_heading;
    if (!std::isfinite(S) || S < 1e-9f)
    {
        if (publish_nis)
        {
            m_magNis = std::numeric_limits<float>::quiet_NaN();
            ++m_magNisCount;
            m_magNisTimestampUs = micros();
            m_magNisRejectReason = SensorStructs::NisRejectReason::NUMERIC_FAILURE;
            m_magNisRejectTimestampUs = m_magNisTimestampUs;
        }
        return;
    }
    const float mag_nis = heading_error * heading_error / S;
    const bool innovation_accepted = std::isfinite(mag_nis) &&
                                     mag_nis <= nisGateThreshold(1);
    if (publish_nis)
    {
        m_magNis = mag_nis;
        ++m_magNisCount;
        m_magNisTimestampUs = micros();
        m_magNisRejectReason = innovation_accepted
            ? SensorStructs::NisRejectReason::NONE
            : (std::isfinite(mag_nis)
                ? SensorStructs::NisRejectReason::INNOVATION_GATE
                : SensorStructs::NisRejectReason::NUMERIC_FAILURE);
        m_magNisRejectTimestampUs = innovation_accepted ? 0 : m_magNisTimestampUs;
    }
    if (!innovation_accepted)
    {
        return;
    }

    const Eigen::Matrix<float, 16, 1> K_heading = m_P * H_heading.transpose() / S;
    const float yaw_correction = (heading_variance / S) * heading_error;

    Eigen::Quaternionf q_new =
        Eigen::Quaternionf(Eigen::AngleAxisf(yaw_correction, Vec3::UnitZ())) *
        q_body_to_ned;
    q_new.normalize();
    if (q_new.w() < 0.0f) {
        q_new.coeffs() *= -1.0f;
    }
    m_x.segment<4>(6) << q_new.w(), q_new.x(), q_new.y(), q_new.z();

    m_IKH.noalias()    = Eigen::Matrix<float,16,16>::Identity() - K_heading * H_heading;
    m_P_temp.noalias() = m_IKH * m_P;
    m_P.noalias()      = m_P_temp * m_IKH.transpose();
    m_P_temp.noalias() = K_heading * R_heading * K_heading.transpose();
    m_P               += m_P_temp;
    m_P_temp           = m_P + m_P.transpose();
    m_P                = 0.5f * m_P_temp;

}

EKF::CorrectionScheduleState EKF::captureScheduleState() const
{
    CorrectionScheduleState state;
    state.lastAccelCorrectionTime = m_lastAccelCorrectionTime;
    state.lastMagCorrectionTime = m_lastMagCorrectionTime;
    state.lastBaroCorrectionTime = m_lastBaroCorrectionTime;
    state.lastGpsCorrectionTime = m_lastGpsCorrectionTime;
    state.lastLidarCorrectionTime = m_lastLidarCorrectionTime;
    state.lastRtkCorrectionTime = m_lastRtkCorrectionTime;
    state.lastMagMeasurementTime = m_lastMagMeasurementTime;
    state.lastBaroMeasurementTime = m_lastBaroMeasurementTime;
    state.lastGpsMeasurementTime = m_lastGpsMeasurementTime;
    state.lastLidarMeasurementTime = m_lastLidarMeasurementTime;
    state.lastRtkMeasurementTime = m_lastRtkMeasurementTime;
    state.lastRtkMeasurementEpochMs = m_lastRtkMeasurementEpochMs;
    state.nextCorrectionIndex = m_nextCorrectionIndex;
    return state;
}

void EKF::restoreScheduleState(const CorrectionScheduleState& state)
{
    m_lastAccelCorrectionTime = state.lastAccelCorrectionTime;
    m_lastMagCorrectionTime = state.lastMagCorrectionTime;
    m_lastBaroCorrectionTime = state.lastBaroCorrectionTime;
    m_lastGpsCorrectionTime = state.lastGpsCorrectionTime;
    m_lastLidarCorrectionTime = state.lastLidarCorrectionTime;
    m_lastRtkCorrectionTime = state.lastRtkCorrectionTime;
    m_lastMagMeasurementTime = state.lastMagMeasurementTime;
    m_lastBaroMeasurementTime = state.lastBaroMeasurementTime;
    m_lastGpsMeasurementTime = state.lastGpsMeasurementTime;
    m_lastLidarMeasurementTime = state.lastLidarMeasurementTime;
    m_lastRtkMeasurementTime = state.lastRtkMeasurementTime;
    m_lastRtkMeasurementEpochMs = state.lastRtkMeasurementEpochMs;
    m_nextCorrectionIndex = state.nextCorrectionIndex;
}

bool EKF::initialiseAttitudeIfSettled(const Eigen::Vector3f& gyro,
                                      const Eigen::Vector3f& accel,
                                      const SensorStructs::MAG_3AXIS_t& mag)
{
    // The estimator runs faster than the magnetometer. Only let each physical
    // magnetometer sample contribute once to startup alignment.
    if (!isNewMeasurement(mag.timestamp_us, m_lastAttitudeInitMagMeasurementTime))
    {
        return false;
    }
    m_lastAttitudeInitMagMeasurementTime = mag.timestamp_us;

    const Eigen::Vector3f rates = gyro - m_x.segment<3>(13);
    const Eigen::Vector3f accel_body = accel - m_x.segment<3>(10);
    const Eigen::Vector3f mag_body(mag.mx, mag.my, mag.mz);

    const float accel_norm = accel_body.norm();
    const float mag_norm = mag_body.norm();
    const bool settled =
        isFiniteVector(rates) &&
        isFiniteVector(accel_body) &&
        isFiniteVector(mag_body) &&
        accel_norm > 1e-6f &&
        std::abs(accel_norm - g) <= ATTITUDE_INIT_ACCEL_GATE &&
        rates.norm() <= ATTITUDE_INIT_GYRO_MAX_RAD_S &&
        mag.timestamp_us != 0 &&
        mag_norm > 1e-6f;

    if (!settled)
    {
        m_attitudeInitSampleCount = 0;
        m_attitudeInitAccelAccum.setZero();
        m_attitudeInitMagAccum.setZero();
        return false;
    }

    m_attitudeInitAccelAccum += accel_body;
    m_attitudeInitMagAccum += mag_body;
    m_attitudeInitSampleCount++;

    if (m_attitudeInitSampleCount < ATTITUDE_INIT_SETTLED_SAMPLE_COUNT)
    {
        return false;
    }

    const float sample_count = static_cast<float>(m_attitudeInitSampleCount);
    const Eigen::Vector3f accel_average = m_attitudeInitAccelAccum / sample_count;
    const Eigen::Vector3f mag_average = m_attitudeInitMagAccum / sample_count;

    Eigen::Quaternionf q_body_to_ned;
    if (!buildInitialAttitude(accel_average, mag_average, q_body_to_ned))
    {
        m_attitudeInitSampleCount = 0;
        m_attitudeInitAccelAccum.setZero();
        m_attitudeInitMagAccum.setZero();
        return false;
    }

    m_x.segment<4>(6) << q_body_to_ned.w(),
                         q_body_to_ned.x(),
                         q_body_to_ned.y(),
                         q_body_to_ned.z();
    m_P.block<4,4>(6,6) = 0.05f * Eigen::Matrix<float,4,4>::Identity();

    const Eigen::Matrix3f R_body_to_ned = q_body_to_ned.toRotationMatrix();
    m_h.segment<3>(0) = R_body_to_ned.transpose() * m_mag_ref.normalized();
    m_h.segment<3>(3) = R_body_to_ned.transpose() * Eigen::Vector3f(0.0f, 0.0f, -g)
                        + m_x.segment<3>(10);
    m_y.segment<3>(0).setZero();
    m_y.segment<3>(3).setZero();

    if constexpr (DebugConfig::EkfAttitudeInitPrintEnabled)
    {
        Serial.printf(
            "EKF_ATT_INIT samples=%u accel_body=(%.3f,%.3f,%.3f) mag_body=(%.3f,%.3f,%.3f) "
            "mag_ref_ned=(%.3f,%.3f,%.3f) body_x_world=(%.3f,%.3f,%.3f) "
            "body_y_world=(%.3f,%.3f,%.3f) body_z_world=(%.3f,%.3f,%.3f)\n",
            static_cast<unsigned>(m_attitudeInitSampleCount),
            accel_average(0), accel_average(1), accel_average(2),
            mag_average(0), mag_average(1), mag_average(2),
            m_mag_ref(0), m_mag_ref(1), m_mag_ref(2),
            R_body_to_ned(0, 0), R_body_to_ned(1, 0), R_body_to_ned(2, 0),
            R_body_to_ned(0, 1), R_body_to_ned(1, 1), R_body_to_ned(2, 1),
            R_body_to_ned(0, 2), R_body_to_ned(1, 2), R_body_to_ned(2, 2));
    }

    m_attitudeInitialised = true;
    // Startup alignment already consumed this magnetometer sample.
    m_lastMagMeasurementTime = m_lastAttitudeInitMagMeasurementTime;
    m_attitudeInitSampleCount = 0;
    m_attitudeInitAccelAccum.setZero();
    m_attitudeInitMagAccum.setZero();
    return true;
}

bool EKF::buildInitialAttitude(const Eigen::Vector3f& accel_body,
                               const Eigen::Vector3f& mag_body,
                               Eigen::Quaternionf& q_body_to_ned) const
{
    if (!isFiniteVector(accel_body) ||
        !isFiniteVector(mag_body) ||
        !isFiniteVector(m_mag_ref))
    {
        return false;
    }

    const float accel_norm = accel_body.norm();
    const float mag_norm = mag_body.norm();
    const float mag_ref_norm = m_mag_ref.norm();
    if (accel_norm < 1e-6f || mag_norm < 1e-6f || mag_ref_norm < 1e-6f)
    {
        return false;
    }

    const Eigen::Vector3f body_up = accel_body / accel_norm;
    const Eigen::Vector3f ned_up(0.0f, 0.0f, -1.0f);
    const Eigen::Vector3f mag_unit = mag_body / mag_norm;
    const Eigen::Vector3f mag_ref_unit = m_mag_ref / mag_ref_norm;

    Eigen::Vector3f body_mag_horizontal =
        mag_unit - body_up * mag_unit.dot(body_up);
    Eigen::Vector3f ned_mag_horizontal =
        mag_ref_unit - ned_up * mag_ref_unit.dot(ned_up);

    const float body_mag_horizontal_norm = body_mag_horizontal.norm();
    const float ned_mag_horizontal_norm = ned_mag_horizontal.norm();
    if (body_mag_horizontal_norm < 1e-6f || ned_mag_horizontal_norm < 1e-6f)
    {
        return false;
    }

    body_mag_horizontal /= body_mag_horizontal_norm;
    ned_mag_horizontal /= ned_mag_horizontal_norm;

    Eigen::Vector3f body_cross = body_up.cross(body_mag_horizontal);
    Eigen::Vector3f ned_cross = ned_up.cross(ned_mag_horizontal);
    const float body_cross_norm = body_cross.norm();
    const float ned_cross_norm = ned_cross.norm();
    if (body_cross_norm < 1e-6f || ned_cross_norm < 1e-6f)
    {
        return false;
    }

    body_cross /= body_cross_norm;
    ned_cross /= ned_cross_norm;

    Eigen::Matrix3f body_basis;
    body_basis.col(0) = body_up;
    body_basis.col(1) = body_mag_horizontal;
    body_basis.col(2) = body_cross;

    Eigen::Matrix3f ned_basis;
    ned_basis.col(0) = ned_up;
    ned_basis.col(1) = ned_mag_horizontal;
    ned_basis.col(2) = ned_cross;

    const Eigen::Matrix3f R_body_to_ned = ned_basis * body_basis.transpose();
    q_body_to_ned = Eigen::Quaternionf(R_body_to_ned);
    q_body_to_ned.normalize();
    if (q_body_to_ned.w() < 0.0f)
    {
        q_body_to_ned.coeffs() *= -1.0f;
    }

    return std::isfinite(q_body_to_ned.w()) &&
           std::isfinite(q_body_to_ned.x()) &&
           std::isfinite(q_body_to_ned.y()) &&
           std::isfinite(q_body_to_ned.z());
}

void EKF::resetHistory()
{
    for (HistorySample& sample : m_history)
    {
        sample.valid = false;
    }
    m_historyHead = 0;
    m_historyCount = 0;
}

void EKF::saveHistorySample(const uint32_t now,
                            const float dt,
                            const float covariance_dt,
                            const bool propagate_covariance,
                            const Eigen::Vector3f& gyro,
                            const Eigen::Vector3f& accel,
                            const Eigen::Vector3f& h_accel,
                            const SensorStructs::MAG_3AXIS_t& mag,
                            const SensorStructs::BARO_t& baro,
                            const SensorStructs::GPS_t& gps,
                            const SensorStructs::LIDAR_t& lidar,
                            const SensorStructs::RTK_t& rtk)
{
    HistorySample& sample = m_history[m_historyHead];
    sample.valid = true;
    sample.timestamp_us = now;
    sample.dt = dt;
    sample.covariance_dt = covariance_dt;
    sample.propagate_covariance = propagate_covariance;
    sample.gyro = gyro;
    sample.accel = accel;
    sample.h_accel = h_accel;
    sample.mag = mag;
    sample.baro = baro;
    sample.gps = gps;
    sample.lidar = lidar;
    sample.rtk = rtk;
    sample.x = m_x;
    sample.P = m_P;
    sample.schedule = captureScheduleState();

    m_historyHead = (m_historyHead + 1) % HISTORY_SAMPLE_COUNT;
    if (m_historyCount < HISTORY_SAMPLE_COUNT)
    {
        m_historyCount++;
    }
}

void EKF::overwriteLatestHistoryState()
{
    const int latest = latestHistoryIndex();
    if (latest < 0)
    {
        return;
    }

    HistorySample& sample = m_history[static_cast<size_t>(latest)];
    sample.x = m_x;
    sample.P = m_P;
    sample.schedule = captureScheduleState();
}

int EKF::latestHistoryIndex() const
{
    if (m_historyCount == 0)
    {
        return -1;
    }

    return static_cast<int>((m_historyHead + HISTORY_SAMPLE_COUNT - 1) % HISTORY_SAMPLE_COUNT);
}

int EKF::nextHistoryIndex(const int index) const
{
    if (index < 0)
    {
        return -1;
    }

    const size_t next = (static_cast<size_t>(index) + 1) % HISTORY_SAMPLE_COUNT;
    if (!m_history[next].valid)
    {
        return -1;
    }

    return static_cast<int>(next);
}

int EKF::findHistoryIndexAtOrBefore(const uint32_t timestamp_us) const
{
    if (m_historyCount == 0)
    {
        return -1;
    }

    const size_t oldest = (m_historyHead + HISTORY_SAMPLE_COUNT - m_historyCount) % HISTORY_SAMPLE_COUNT;
    int best = -1;
    for (size_t i = 0; i < m_historyCount; i++)
    {
        const size_t index = (oldest + i) % HISTORY_SAMPLE_COUNT;
        const HistorySample& sample = m_history[index];
        if (!sample.valid)
        {
            continue;
        }

        if (timeAtOrAfter(timestamp_us, sample.timestamp_us))
        {
            best = static_cast<int>(index);
        }
        else
        {
            break;
        }
    }

    return best;
}

void EKF::updateGnssTimeOffset(const SensorStructs::GPS_t& gps)
{
    if (!gps.valid || gps.timestamp_us == 0 || gps.gnss_time_of_day_ms == 0)
    {
        return;
    }

    const int64_t gnss_us = static_cast<int64_t>(gps.gnss_time_of_day_ms) * 1000LL;
    const int64_t candidate_offset = static_cast<int64_t>(gps.timestamp_us) - gnss_us;
    if (!m_gnssTimeOffsetValid)
    {
        m_gnssToLocalOffsetUs = candidate_offset;
        m_gnssTimeOffsetValid = true;
        return;
    }

    int64_t error = candidate_offset - m_gnssToLocalOffsetUs;
    const int64_t half_day_us = static_cast<int64_t>(GNSS_DAY_US / 2ULL);
    const int64_t day_us = static_cast<int64_t>(GNSS_DAY_US);
    while (error > half_day_us)  { error -= day_us; }
    while (error < -half_day_us) { error += day_us; }

    m_gnssToLocalOffsetUs += error / 8;
}

bool EKF::gnssTimeOfDayToLocalUs(const uint32_t gnss_time_of_day_ms,
                                 const uint32_t now,
                                 uint32_t& local_us) const
{
    if (!m_gnssTimeOffsetValid || gnss_time_of_day_ms == 0)
    {
        return false;
    }

    int64_t local = static_cast<int64_t>(gnss_time_of_day_ms) * 1000LL + m_gnssToLocalOffsetUs;
    const int64_t now64 = static_cast<int64_t>(now);
    const int64_t half_day_us = static_cast<int64_t>(GNSS_DAY_US / 2ULL);
    const int64_t day_us = static_cast<int64_t>(GNSS_DAY_US);
    while (now64 - local > half_day_us) { local += day_us; }
    while (local - now64 > half_day_us) { local -= day_us; }

    local_us = static_cast<uint32_t>(local);
    return true;
}

bool EKF::applyGnssTimestamp(SensorStructs::RTK_t& rtk, const uint32_t now) const
{
    if (rtk.gnss_time_of_day_ms == 0)
    {
        rtk.measurement_timestamp_us = 0;
        return false;
    }

    const uint32_t gps_time_of_day_ms =
        (rtk.gnss_time_of_day_ms + GPS_UTC_OFFSET_MS) % GNSS_DAY_MS; // RTK time is UTC; GPS/PPS timing uses GPS time, currently UTC+18s.

    uint32_t measurement_us = 0;
    if (!gnssTimeOfDayToLocalUs(gps_time_of_day_ms, now, measurement_us))
    {
        rtk.measurement_timestamp_us = 0;
        return false;
    }

    rtk.measurement_timestamp_us = measurement_us;
    return true;
}

bool EKF::handleGpsCorrection(const uint32_t now, const SensorStructs::GPS_t& gps)
{
    if (gps.timestamp_us == 0 ||
        (m_lastHandledGpsTimestampUs != 0 &&
         !timeAfter(gps.timestamp_us, m_lastHandledGpsTimestampUs)))
    {
        return false;
    }

    const bool in_past = timeAtOrAfter(now, gps.timestamp_us); // True when GPS time is not ahead of now.
    const uint32_t delay_us = in_past ? now - gps.timestamp_us : 0;
    const bool fresh = in_past &&
                       delay_us <= TimingConfig::EKF::GPS_CORRECTION_MAX_AGE_US;
    if (!fresh)
    {
        m_gpsNisRejectReason = in_past
            ? SensorStructs::NisRejectReason::STALE_MEASUREMENT
            : SensorStructs::NisRejectReason::FUTURE_MEASUREMENT;
        m_gpsNisRejectTimestampUs = micros();
        m_lastGpsMeasurementTime = gps.timestamp_us;
        m_lastHandledGpsTimestampUs = gps.timestamp_us;
        return false;
    }

    if (!correctionDueNow(now, m_lastGpsCorrectionTime, TimingConfig::EKF::GPS_CORRECTION_DELTA_US)) // Rate-limit GPS fusion.
    {
        return false;
    }

    const bool replayed_to_now = fuseDelayedGPS(gps, now); // Rewind, fuse at GPS time, then replay to now.
    if (!replayed_to_now)
    {
        updateGPS(gps); // Fallback: fuse GPS into the current EKF state.
    }

    m_lastGpsCorrectionTime = now;
    m_lastGpsMeasurementTime = gps.timestamp_us;
    m_lastHandledGpsTimestampUs = gps.timestamp_us;
    if (m_lastFusedGpsTimestampUs == 0 ||
        timeAfter(gps.timestamp_us, m_lastFusedGpsTimestampUs))
    {
        m_lastFusedGpsTimestampUs = gps.timestamp_us;
    }
    return replayed_to_now;
}

bool EKF::fuseDelayedGPS(const SensorStructs::GPS_t& gps, const uint32_t now)
{
    const uint32_t measurement_us = gps.timestamp_us;
    if (measurement_us == 0) //check the gps reading is valid
    {
        return false;
    }

    if (timeAtOrAfter(measurement_us, now)) //check if the GPS is in the future, if so, don't fuse it
    {
        return false;
    }

    if (now - measurement_us > TimingConfig::EKF::DELAYED_MEASUREMENT_HISTORY_US) //check if the GPS is too old, if so, don't fuse it
    {
        return false;
    }

    const int base_index = findHistoryIndexAtOrBefore(measurement_us); //find out where in the history the GPS measurement should be fused
    const int latest_index = latestHistoryIndex(); //find out where the latest history sample is in the history with units of microseconds
    if (base_index < 0 || latest_index < 0) //check if it is valid to fuse the GPS measurement, if not, don't fuse it
    {
        return false;
    }

    HistorySample& base = m_history[static_cast<size_t>(base_index)]; //get the history sample at the base index, in microseconds
    m_x = base.x; //find out what the state was at the base index
    m_P = base.P; // find out what the covariance was at the base 
    restoreScheduleState(base.schedule); //restore the schedule state at the base index time

    bool gps_fused = false;
    uint32_t current_time = base.timestamp_us;
    if (timeAtOrAfter(current_time, measurement_us)) // check if the current time is after the GPS measurement time, if so, fuse the GPS measurement
    {
        updateGPS(gps);
        gps_fused = true;
        // Persist the correction identity at the point where it enters history.
        // Otherwise a later RTK rewind can start from a GPS-corrected state but
        // see a pre-GPS schedule and apply the same GPS sample again.
        m_lastGpsCorrectionTime = measurement_us;
        m_lastGpsMeasurementTime = gps.timestamp_us;
        base.x = m_x;
        base.P = m_P;
        base.schedule = captureScheduleState();
    }

    int index = base_index;
    while (index != latest_index)
    {
        const int next_index = nextHistoryIndex(index);
        if (next_index < 0)
        {
            break;
        }

        HistorySample& sample = m_history[static_cast<size_t>(next_index)];
        if (!gps_fused && timeAtOrAfter(sample.timestamp_us, measurement_us))
        {
            const uint32_t partial_us = measurement_us - current_time;
            const float partial_dt = static_cast<float>(partial_us) * 1e-6f;
            if (partial_dt > 0.0f && partial_dt <= 0.5f)
            {
                predict(partial_dt, partial_dt, false, sample.gyro, sample.accel, sample.h_accel);
            }

            updateGPS(gps);
            gps_fused = true;
            m_lastGpsCorrectionTime = measurement_us;
            m_lastGpsMeasurementTime = gps.timestamp_us;

            const uint32_t remaining_us = sample.timestamp_us - measurement_us;
            const float remaining_dt = static_cast<float>(remaining_us) * 1e-6f;
            if (remaining_dt > 0.0f && remaining_dt <= 0.5f)
            {
                predict(remaining_dt,
                        sample.covariance_dt,
                        sample.propagate_covariance,
                        sample.gyro,
                        sample.accel,
                        sample.h_accel);
            }
        }
        else
        {
            predict(sample.dt,
                    sample.covariance_dt,
                    sample.propagate_covariance,
                    sample.gyro,
                    sample.accel,
                    sample.h_accel);
        }

        // Replay every scheduler opportunity. The live path no longer skips
        // correction work when covariance propagation is due, so doing so here
        // would erase corrections (and disproportionately starve lidar).
        runScheduledCorrection(sample.timestamp_us,
                               sample.accel,
                               sample.mag,
                               sample.baro,
                               sample.gps,
                               sample.lidar,
                               sample.rtk,
                               false,
                               true);

        // Persist the replayed state at every history timestamp. A later
        // delayed correction may rewind to any of these samples; leaving an
        // intermediate sample unchanged would discard corrections applied by
        // this replay.
        sample.x = m_x;
        sample.P = m_P;
        sample.schedule = captureScheduleState();

        current_time = sample.timestamp_us;
        index = next_index;
    }

    return gps_fused;
}

bool EKF::handleRtkCorrection(const uint32_t now, const SensorStructs::RTK_t& rtk)
{
    if (rtk.timestamp_us == 0)
    {
        return false;
    }

    const bool duplicate_source = m_lastHandledRtkTimestampUs != 0 &&
                                  !timeAfter(rtk.timestamp_us, m_lastHandledRtkTimestampUs);
    const bool duplicate_or_old_epoch = rtk.gnss_time_of_day_ms != 0 &&
                                        m_lastHandledRtkEpochMs != 0 &&
                                        !gnssEpochAfter(rtk.gnss_time_of_day_ms,
                                                        m_lastHandledRtkEpochMs);
    if (duplicate_source || duplicate_or_old_epoch)
    {
        // A sender can retransmit a handled GNSS epoch with a new local receive
        // timestamp. Advance only the source watermark without fusing it again.
        if (m_lastHandledRtkTimestampUs == 0 ||
            timeAfter(rtk.timestamp_us, m_lastHandledRtkTimestampUs))
        {
            m_lastHandledRtkTimestampUs = rtk.timestamp_us;
        }
        return false;
    }

    // Invalid or unusable packets are consumed too, so a retained packet is
    // diagnosed once instead of being reconsidered on every estimator tick.
    if (!rtk.valid || rtk.fix_quality == 0)
    {
        m_rtkNisRejectReason = SensorStructs::NisRejectReason::SENSOR_QUALITY_GATE;
        m_rtkNisRejectTimestampUs = micros();
        m_lastHandledRtkTimestampUs = rtk.timestamp_us;
        m_lastHandledRtkEpochMs = rtk.gnss_time_of_day_ms;
        return false;
    }
    if (!rtk.home_set)
    {
        m_rtkNisRejectReason = SensorStructs::NisRejectReason::NO_HOME_REFERENCE;
        m_rtkNisRejectTimestampUs = micros();
        m_lastHandledRtkTimestampUs = rtk.timestamp_us;
        m_lastHandledRtkEpochMs = rtk.gnss_time_of_day_ms;
        return false;
    }

    SensorStructs::RTK_t timestamped_rtk = rtk;
    const bool has_measurement_time = applyGnssTimestamp(timestamped_rtk, now);
    const uint32_t correction_timestamp_us = has_measurement_time
        ? timestamped_rtk.measurement_timestamp_us
        : timestamped_rtk.timestamp_us;
    const uint32_t delay_us = correction_timestamp_us != 0
        ? now - correction_timestamp_us
        : 0;
    const bool in_past = correction_timestamp_us != 0 &&
                         timeAtOrAfter(now, correction_timestamp_us);
    const bool fresh = in_past &&
                       delay_us <= TimingConfig::EKF::RTK_CORRECTION_MAX_AGE_US;

    if (correction_timestamp_us == 0)
    {
        return false;
    }

    if (!fresh)
    {
        m_rtkNisRejectReason = in_past
            ? SensorStructs::NisRejectReason::STALE_MEASUREMENT
            : SensorStructs::NisRejectReason::FUTURE_MEASUREMENT;
        m_rtkNisRejectTimestampUs = micros();
        m_lastRtkMeasurementTime = rtk.timestamp_us;
        m_lastRtkMeasurementEpochMs = rtk.gnss_time_of_day_ms;
        m_lastHandledRtkTimestampUs = rtk.timestamp_us;
        m_lastHandledRtkEpochMs = rtk.gnss_time_of_day_ms;
        return false;
    }

    m_lastRtkDelayUs = delay_us;

    if (m_lastRtkCorrectionTime != 0 &&
        now - m_lastRtkCorrectionTime < TimingConfig::EKF::RTK_CORRECTION_DELTA_US)
    {
        return false;
    }

    const bool replayed_to_now = fuseDelayedRTK(timestamped_rtk, now);
    if (!replayed_to_now)
    {
        updateRTK(timestamped_rtk);
    }

    m_lastRtkCorrectionTime = now;
    m_lastRtkMeasurementTime = rtk.timestamp_us;
    m_lastRtkMeasurementEpochMs = rtk.gnss_time_of_day_ms;
    m_lastHandledRtkTimestampUs = rtk.timestamp_us;
    m_lastHandledRtkEpochMs = rtk.gnss_time_of_day_ms;
    if (m_lastFusedRtkTimestampUs == 0 ||
        timeAfter(rtk.timestamp_us, m_lastFusedRtkTimestampUs))
    {
        m_lastFusedRtkTimestampUs = rtk.timestamp_us;
    }
    return replayed_to_now;
}

bool EKF::fuseDelayedRTK(const SensorStructs::RTK_t& rtk, const uint32_t now)
{
    const uint32_t measurement_us = rtk.measurement_timestamp_us;
    if (measurement_us == 0)
    {
        return false;
    }

    if (timeAtOrAfter(measurement_us, now))
    {
        return false;
    }

    if (now - measurement_us > TimingConfig::EKF::DELAYED_MEASUREMENT_HISTORY_US)
    {
        return false;
    }

    const int base_index = findHistoryIndexAtOrBefore(measurement_us);
    const int latest_index = latestHistoryIndex();
    if (base_index < 0 || latest_index < 0)
    {
        return false;
    }

    HistorySample& base = m_history[static_cast<size_t>(base_index)];
    m_x = base.x;
    m_P = base.P;
    restoreScheduleState(base.schedule);

    bool rtk_fused = false;
    uint32_t current_time = base.timestamp_us;
    if (timeAtOrAfter(current_time, measurement_us))
    {
        SensorStructs::RTK_t delayed_rtk = rtk;
        delayed_rtk.measurement_timestamp_us = measurement_us;
        updateRTK(delayed_rtk);
        rtk_fused = true;
        m_lastRtkCorrectionTime = measurement_us;
        m_lastRtkMeasurementTime = rtk.timestamp_us;
        m_lastRtkMeasurementEpochMs = rtk.gnss_time_of_day_ms;
        base.x = m_x;
        base.P = m_P;
        base.schedule = captureScheduleState();
    }

    int index = base_index;
    while (index != latest_index)
    {
        const int next_index = nextHistoryIndex(index);
        if (next_index < 0)
        {
            break;
        }

        HistorySample& sample = m_history[static_cast<size_t>(next_index)];
        if (!rtk_fused && timeAtOrAfter(sample.timestamp_us, measurement_us))
        {
            const uint32_t partial_us = measurement_us - current_time;
            const float partial_dt = static_cast<float>(partial_us) * 1e-6f;
            if (partial_dt > 0.0f && partial_dt <= 0.5f)
            {
                predict(partial_dt, partial_dt, false, sample.gyro, sample.accel, sample.h_accel);
            }

            SensorStructs::RTK_t delayed_rtk = rtk;
            delayed_rtk.measurement_timestamp_us = measurement_us;
            updateRTK(delayed_rtk);
            rtk_fused = true;
            m_lastRtkCorrectionTime = measurement_us;
            m_lastRtkMeasurementTime = rtk.timestamp_us;
            m_lastRtkMeasurementEpochMs = rtk.gnss_time_of_day_ms;

            const uint32_t remaining_us = sample.timestamp_us - measurement_us;
            const float remaining_dt = static_cast<float>(remaining_us) * 1e-6f;
            if (remaining_dt > 0.0f && remaining_dt <= 0.5f)
            {
                predict(remaining_dt,
                        sample.covariance_dt,
                        sample.propagate_covariance,
                        sample.gyro,
                        sample.accel,
                        sample.h_accel);
            }
        }
        else
        {
            predict(sample.dt,
                    sample.covariance_dt,
                    sample.propagate_covariance,
                    sample.gyro,
                    sample.accel,
                    sample.h_accel);
        }

        runScheduledCorrection(sample.timestamp_us,
                               sample.accel,
                               sample.mag,
                               sample.baro,
                               sample.gps,
                               sample.lidar,
                               sample.rtk,
                               true,
                               false);

        // Persist the replayed state at every history timestamp so subsequent
        // delayed RTK corrections retain all corrections already replayed.
        sample.x = m_x;
        sample.P = m_P;
        sample.schedule = captureScheduleState();

        current_time = sample.timestamp_us;
        index = next_index;
    }

    return rtk_fused;
}

void EKF::runScheduledCorrection(const uint32_t now,
                                 const Eigen::Vector3f& accel,
                                 const SensorStructs::MAG_3AXIS_t& mag,
                                 const SensorStructs::BARO_t& baro,
                                 const SensorStructs::GPS_t& gps,
                                 const SensorStructs::LIDAR_t& lidar,
                                 const SensorStructs::RTK_t& rtk,
                                 const bool allow_gps,
                                 const bool allow_rtk)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        const uint8_t correction = m_nextCorrectionIndex;
        m_nextCorrectionIndex = (m_nextCorrectionIndex + 1) % 6;

        switch (correction)
        {
            case 0:
                if (timerDue(now, m_lastAccelCorrectionTime, TimingConfig::EKF::ACCEL_CORRECTION_DELTA_US))
                {
                    updateLowGAccel(accel, now);
                    return;
                }
                break;
            case 1:
                if (isNewMeasurement(mag.timestamp_us, m_lastMagMeasurementTime) &&
                    timerDue(now, m_lastMagCorrectionTime, TimingConfig::EKF::MAG_CORRECTION_DELTA_US))
                {
                    m_lastMagMeasurementTime = mag.timestamp_us;
                    updateMag(Eigen::Vector3f(mag.mx, mag.my, mag.mz), mag.timestamp_us);
                    return;
                }
                break;
            case 2:
                if (isNewMeasurement(baro.timestamp_us, m_lastBaroMeasurementTime) &&
                    timerDue(now, m_lastBaroCorrectionTime, TimingConfig::EKF::BARO_CORRECTION_DELTA_US))
                {
                    m_lastBaroMeasurementTime = baro.timestamp_us;
                    updateBaro(baro.press, baro.temp, baro.timestamp_us);
                    return;
                }
                break;
            case 3:
                if constexpr (TimingConfig::EKF::GPS_CORRECTION_ENABLED)
                {
                    if (allow_gps &&
                        gps.timestamp_us != 0 &&
                        m_lastFusedGpsTimestampUs != 0 &&
                        timeAtOrAfter(m_lastFusedGpsTimestampUs, gps.timestamp_us) &&
                        isNewMeasurement(gps.timestamp_us, m_lastGpsMeasurementTime) &&
                        timerDue(now, m_lastGpsCorrectionTime, TimingConfig::EKF::GPS_CORRECTION_DELTA_US))
                    {
                        m_lastGpsMeasurementTime = gps.timestamp_us;
                        updateGPS(gps);
                        return;
                    }
                }
                break;
            case 4:
                if (allow_rtk &&
                    rtk.valid &&
                    rtk.home_set &&
                    rtk.fix_quality != 0 &&
                    rtk.timestamp_us != 0 &&
                    m_lastFusedRtkTimestampUs != 0 &&
                    timeAtOrAfter(m_lastFusedRtkTimestampUs, rtk.timestamp_us) &&
                    isNewMeasurement(rtk.timestamp_us, m_lastRtkMeasurementTime) &&
                    (rtk.gnss_time_of_day_ms == 0 ||
                     rtk.gnss_time_of_day_ms != m_lastRtkMeasurementEpochMs))
                {
                    const bool fresh = now - rtk.timestamp_us <= TimingConfig::EKF::RTK_CORRECTION_MAX_AGE_US;
                    if (!fresh)
                    {
                        m_lastRtkMeasurementTime = rtk.timestamp_us;
                        m_lastRtkMeasurementEpochMs = rtk.gnss_time_of_day_ms;
                        break;
                    }

                    if (correctionDueNow(now, m_lastRtkCorrectionTime, TimingConfig::EKF::RTK_CORRECTION_DELTA_US))
                    {
                        m_lastRtkMeasurementTime = rtk.timestamp_us;
                        m_lastRtkMeasurementEpochMs = rtk.gnss_time_of_day_ms;
                        updateRTK(rtk);
                        return;
                    }
                }
                break;
            case 5:
                if constexpr (TimingConfig::EKF::LIDAR_CORRECTION_ENABLED)
                {
                    if (isNewMeasurement(lidar.timestamp_us, m_lastLidarMeasurementTime) &&
                        timerDue(now, m_lastLidarCorrectionTime, TimingConfig::EKF::LIDAR_CORRECTION_DELTA_US))
                    {
                        m_lastLidarMeasurementTime = lidar.timestamp_us;
                        updateLidar(lidar);
                        return;
                    }
                }
                break;
            default:
                break;
        }
    }
}

void EKF::updateLowGAccel(const Eigen::Vector3f& z_accel,
                          const uint32_t measurement_time_us)
{
    using Mat3 = Eigen::Matrix3f;
    using Vec3 = Eigen::Vector3f;
    using Vec4 = Eigen::Vector4f;

    const bool publish_nis = isNewMeasurement(measurement_time_us,
                                               m_lastAccelNisMeasurementTime);
    if (publish_nis)
    {
        m_lastAccelNisMeasurementTime = measurement_time_us;
    }

    const float z_norm = z_accel.norm();
    if (!std::isfinite(z_norm) || z_norm < 1e-6f) {
        if (publish_nis)
        {
            m_accelNisRejectReason = SensorStructs::NisRejectReason::INVALID_MEASUREMENT;
            m_accelNisRejectTimestampUs = micros();
        }
        return;
    }

    const float accel_error = std::abs(z_norm - g);

    // Hard reject only when accel is clearly not gravity-dominated
    if (accel_error > ACCEL_GATE) {
        if (publish_nis)
        {
            m_accelNisRejectReason = SensorStructs::NisRejectReason::ACCELERATION_GATE;
            m_accelNisRejectTimestampUs = micros();
        }
        return;
    }
    // Smoothly reduce accel trust as |a| moves away from 1g
    const float scale = 1.0f + 3.0f * (accel_error / ACCEL_GATE);

    Vec4 q = m_x.segment<4>(6); // get the attitude from the predict step
    if (q.norm() < 1e-9f) { q = Vec4(1.0f, 0.0f, 0.0f, 0.0f); }
    else                  { q.normalize(); }
    const float q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);

    const Vec3 ba_low = m_x.segment<3>(10); //get the bias of the accelerometer
    const Vec3 g_ned(0.0f, 0.0f, -g);

    m_h.segment<3>(3) = Eigen::Quaternionf(q0, q1, q2, q3).toRotationMatrix().transpose() * g_ned + ba_low; //expected readings
    m_y.segment<3>(3) = z_accel - m_h.segment<3>(3); //innovation of the accelerometer


    Eigen::Matrix<float, 3, 4> Hq; // derived assuming (0,0,-1)
    Hq <<    2*q2, -2*q3,   2*q0,  -2*q1,
            -2*q1, -2*q0,  -2*q3,  -2*q2,
                0,  4*q1,   4*q2,      0;
    Hq *= g;

    m_H.setZero();
    m_H.block<3,4>(0,6)  = Hq;
    m_H.block<3,3>(0,10) = Mat3::Identity();

    const Mat3 R_base = SIGMA_ACCEL_LOW.cwiseProduct(SIGMA_ACCEL_LOW).asDiagonal(); 
    const Mat3 R = scale * R_base; //here is where we reduce the trust in accel as it moves away from 1g

    const Mat3 S = m_H * m_P * m_H.transpose() + R;
    const Eigen::LDLT<Mat3> S_ldlt(S);
    const Vec3 innovation = m_y.segment<3>(3);
    float accel_nis = std::numeric_limits<float>::quiet_NaN();
    const bool nis_valid = calculateNis(S_ldlt, innovation, accel_nis);
    const bool innovation_accepted = nis_valid &&
                                     accel_nis <= nisGateThreshold(3);
    if (publish_nis)
    {
        m_accelNis = accel_nis;
        ++m_accelNisCount;
        m_accelNisTimestampUs = micros();
        m_accelNisRejectReason = innovation_accepted
            ? SensorStructs::NisRejectReason::NONE
            : (nis_valid
                ? SensorStructs::NisRejectReason::INNOVATION_GATE
                : SensorStructs::NisRejectReason::NUMERIC_FAILURE);
        m_accelNisRejectTimestampUs = innovation_accepted ? 0 : m_accelNisTimestampUs;
    }
    if (!innovation_accepted)
    {
        return;
    }
    m_K = m_P * m_H.transpose() * S_ldlt.solve(Mat3::Identity());

   

    m_K.block<6,3>(0,0).setZero();   // position/vel
    m_x += m_K * m_y.segment<3>(3); //update the state

    m_IKH.noalias()    = Eigen::Matrix<float,16,16>::Identity() - m_K * m_H; //update the covariance
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

void EKF::updateBaro(const float pressure,
                     const float temperature,
                     const uint32_t measurement_time_us)
{
    using Mat2 = Eigen::Matrix<float, 2, 2>;
    using Vec2 = Eigen::Matrix<float, 2, 1>;

    const bool publish_nis = isNewMeasurement(measurement_time_us,
                                               m_lastBaroNisMeasurementTime);
    if (publish_nis)
    {
        m_lastBaroNisMeasurementTime = measurement_time_us;
    }

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
    m_y(6) = temperature - m_h(6);
    m_y(7) = pressure - m_h(7);

    // ── Measurement noise ─────────────────────────────────────────────────────
    Mat2 R_baro = Mat2::Zero();
    R_baro(0,0) = SIGMA_T * SIGMA_T;
    R_baro(1,1) = SIGMA_P * SIGMA_P;

    // ── Kalman gain (16×2) ────────────────────────────────────────────────────
    const Mat2 S = H_baro * m_P * H_baro.transpose() + R_baro;
    const Eigen::LDLT<Mat2> S_ldlt(S);
    const Vec2 innovation = m_y.segment<2>(6);
    float baro_nis = std::numeric_limits<float>::quiet_NaN();
    const bool nis_valid = calculateNis(S_ldlt, innovation, baro_nis);
    const bool innovation_accepted = nis_valid &&
                                     baro_nis <= nisGateThreshold(2);
    if (publish_nis)
    {
        m_baroNis = baro_nis;
        ++m_baroNisCount;
        m_baroNisTimestampUs = micros();
        m_baroNisRejectReason = innovation_accepted
            ? SensorStructs::NisRejectReason::NONE
            : (nis_valid
                ? SensorStructs::NisRejectReason::INNOVATION_GATE
                : SensorStructs::NisRejectReason::NUMERIC_FAILURE);
        m_baroNisRejectTimestampUs = innovation_accepted ? 0 : m_baroNisTimestampUs;
    }
    if (!innovation_accepted)
    {
        return;
    }
    const Eigen::Matrix<float, 16, 2> K_baro = m_P * H_baro.transpose() * S_ldlt.solve(Mat2::Identity());

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

    const bool publish_nis = gps.timestamp_us != 0 &&
                             (m_lastGpsNisMeasurementTime == 0 ||
                              timeAfter(gps.timestamp_us, m_lastGpsNisMeasurementTime));
    if (publish_nis)
    {
        m_lastGpsNisMeasurementTime = gps.timestamp_us;
    }

    if constexpr (!TimingConfig::EKF::GPS_CORRECTION_ENABLED)
    {
        if (publish_nis)
        {
            m_gpsNisRejectReason = SensorStructs::NisRejectReason::NIS_DISABLED;
            m_gpsNisRejectTimestampUs = 0;
        }
        return;
    }

    // ── Quality gate ──────────────────────────────────────────────────────────
    if (!gps.valid || gps.fix < 1 || gps.sat < 4)
    {
        if (publish_nis)
        {
            m_gpsNisRejectReason = SensorStructs::NisRejectReason::SENSOR_QUALITY_GATE;
            m_gpsNisRejectTimestampUs = micros();
        }
        return;
    }

    const bool pos_valid = USE_GPS_POSITION &&
                           std::isfinite(gps.hAcc) &&
                           gps.hAcc > 0.0f &&
                           gps.hAcc <= 3.0f;
    const bool vel_valid = std::isfinite(gps.v_n) &&
                           std::isfinite(gps.v_e) &&
                           std::isfinite(gps.v_d);

    if (!pos_valid && !vel_valid)
    {
        if (publish_nis)
        {
            m_gpsNisRejectReason = SensorStructs::NisRejectReason::INVALID_MEASUREMENT;
            m_gpsNisRejectTimestampUs = micros();
        }
        return;
    }

    if (pos_valid) {
        // ── Convert to radians in double ──────────────────────────────────────
        const double lat0 = static_cast<double>(m_setHome_ref.launch_lat) * 1e-7 * M_PI / 180.0;
        const double lon0 = static_cast<double>(m_setHome_ref.launch_lon) * 1e-7 * M_PI / 180.0;
        const double lat  = static_cast<double>(gps.latitude)             * 1e-7 * M_PI / 180.0;
        const double lon  = static_cast<double>(gps.longitude)            * 1e-7 * M_PI / 180.0;
        const double h0   = static_cast<double>(m_setHome_ref.launch_alt);
        const double h    = static_cast<double>(gps.altitude);

        // ── LLA -> ECEF (measurement) ─────────────────────────────────────────
        const double N  = GPS_A_EARTH / std::sqrt(1.0 - GPS_E2 * std::sin(lat) * std::sin(lat));
        const double X  = (N + h)                   * std::cos(lat) * std::cos(lon);
        const double Y  = (N + h)                   * std::cos(lat) * std::sin(lon);
        const double Z  = (N * (1.0 - GPS_E2) + h)  * std::sin(lat);

        // ── LLA -> ECEF (reference) ───────────────────────────────────────────
        const double N0 = GPS_A_EARTH / std::sqrt(1.0 - GPS_E2 * std::sin(lat0) * std::sin(lat0));
        const double X0 = (N0 + h0)                   * std::cos(lat0) * std::cos(lon0);
        const double Y0 = (N0 + h0)                   * std::cos(lat0) * std::sin(lon0);
        const double Z0 = (N0 * (1.0 - GPS_E2) + h0)  * std::sin(lat0);

        // ── ECEF delta -> NED ─────────────────────────────────────────────────
        const double dX = X - X0;
        const double dY = Y - Y0;
        const double dZ = Z - Z0;

        m_gps_position(0) = static_cast<float>(-std::sin(lat0)*std::cos(lon0)*dX - std::sin(lat0)*std::sin(lon0)*dY + std::cos(lat0)*dZ);
        m_gps_position(1) = static_cast<float>(-std::sin(lon0)*dX               + std::cos(lon0)*dY);
        m_gps_position(2) = static_cast<float>(-std::cos(lat0)*std::cos(lon0)*dX - std::cos(lat0)*std::sin(lon0)*dY - std::sin(lat0)*dZ);
    }

    // ── Velocity measurement (already in NED from GPS driver) ─────────────────
    const Vec3 z_vel(gps.v_n, gps.v_e, gps.v_d);

    if (!USE_GPS_POSITION && USE_GPS_VELOCITY_DIRECT) {
        m_h.segment<3>(11) = m_x.segment<3>(3);
        m_y.segment<3>(8).setZero();
        m_y.segment<3>(11) = z_vel - m_h.segment<3>(11);

        const Mat3 R_velocity = (SIGMA_VEL * SIGMA_VEL) * Mat3::Identity();
        const Mat3 S = m_P.block<3,3>(3,3) + R_velocity;
        const Eigen::LDLT<Mat3> S_ldlt(S);
        const Vec3 innovation = m_y.segment<3>(11);
        float gps_nis = std::numeric_limits<float>::quiet_NaN();
        const bool nis_valid = calculateNis(S_ldlt, innovation, gps_nis);
        const bool innovation_accepted = nis_valid &&
                                         gps_nis <= nisGateThreshold(3);
        if (publish_nis)
        {
            m_gpsNis = gps_nis;
            ++m_gpsNisCount;
            m_gpsNisTimestampUs = micros();
            m_gpsNisRejectReason = innovation_accepted
                ? SensorStructs::NisRejectReason::NONE
                : (nis_valid
                    ? SensorStructs::NisRejectReason::INNOVATION_GATE
                    : SensorStructs::NisRejectReason::NUMERIC_FAILURE);
            m_gpsNisRejectTimestampUs = innovation_accepted ? 0 : m_gpsNisTimestampUs;
        }
        if (!innovation_accepted)
        {
            return;
        }
        m_x.segment<3>(3) = z_vel;
        return;
    }

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

    const float gps_speed = z_vel.norm();
    const bool use_pos = pos_valid &&
                         gps_speed > 1.0f &&
                         m_y.segment<3>(8).norm() < 5.0f;

    if (!use_pos) {
        H_gps.block<3,3>(0,0).setZero();  // zero out position rows of H
        m_y.segment<3>(8).setZero();
    }

    if (!vel_valid) {
        H_gps.block<3,3>(3,3).setZero();  // zero out velocity rows of H
        m_y.segment<3>(11).setZero();
    }

    const uint8_t active_measurement_dof =
        static_cast<uint8_t>((use_pos ? 3U : 0U) + (vel_valid ? 3U : 0U));
    if (active_measurement_dof == 0)
    {
        if (publish_nis)
        {
            m_gpsNisRejectReason = SensorStructs::NisRejectReason::INVALID_MEASUREMENT;
            m_gpsNisRejectTimestampUs = micros();
        }
        return;
    }

    const Mat6 S    = H_gps * m_P * H_gps.transpose() + R_gps;
    const Eigen::LDLT<Mat6> S_ldlt(S);
    const Vec6 innovation = m_y.segment<6>(8);
    float gps_nis = std::numeric_limits<float>::quiet_NaN();
    const bool nis_valid = calculateNis(S_ldlt, innovation, gps_nis);
    const bool innovation_accepted = nis_valid &&
                                     gps_nis <= nisGateThreshold(active_measurement_dof);
    if (publish_nis)
    {
        m_gpsNis = gps_nis;
        ++m_gpsNisCount;
        m_gpsNisTimestampUs = micros();
        m_gpsNisRejectReason = innovation_accepted
            ? SensorStructs::NisRejectReason::NONE
            : (nis_valid
                ? SensorStructs::NisRejectReason::INNOVATION_GATE
                : SensorStructs::NisRejectReason::NUMERIC_FAILURE);
        m_gpsNisRejectTimestampUs = innovation_accepted ? 0 : m_gpsNisTimestampUs;
    }
    if (!innovation_accepted)
    {
        return;
    }

    // ── Kalman gain (16×6) ────────────────────────────────────────────────────
    const Eigen::Matrix<float, 16, 6> K_gps = m_P * H_gps.transpose() * S_ldlt.solve(Mat6::Identity());

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

void EKF::updateRTK(const SensorStructs::RTK_t& rtk)
{
    using Mat6 = Eigen::Matrix<float, 6, 6>;
    using Vec6 = Eigen::Matrix<float, 6, 1>;

    const bool publish_nis = rtk.timestamp_us != 0 &&
                             (m_lastRtkNisMeasurementTime == 0 ||
                              timeAfter(rtk.timestamp_us, m_lastRtkNisMeasurementTime));
    if (publish_nis)
    {
        m_lastRtkNisMeasurementTime = rtk.timestamp_us;
    }

    if (!rtk.valid || rtk.fix_quality == 0)
    {
        if (publish_nis)
        {
            m_rtkNisRejectReason = SensorStructs::NisRejectReason::SENSOR_QUALITY_GATE;
            m_rtkNisRejectTimestampUs = micros();
        }
        return;
    }

    const Eigen::Vector3f z_pos(rtk.x, rtk.y, rtk.z);
    if (!z_pos.allFinite())
    {
        if (publish_nis)
        {
            m_rtkNisRejectReason = SensorStructs::NisRejectReason::INVALID_MEASUREMENT;
            m_rtkNisRejectTimestampUs = micros();
        }
        return;
    }

    Vec6 innovation = Vec6::Zero();
    innovation.segment<3>(0) = z_pos - m_x.segment<3>(0);

    Eigen::Matrix<float, 6, 16> H_rtk = Eigen::Matrix<float, 6, 16>::Zero();
    H_rtk.block<3,3>(0,0) = Eigen::Matrix3f::Identity();

    if (USE_RTK_VELOCITY)
    {
        const Eigen::Vector3f z_vel(rtk.u, rtk.v, rtk.w);
        if (!z_vel.allFinite())
        {
            if (publish_nis)
            {
                m_rtkNisRejectReason = SensorStructs::NisRejectReason::INVALID_MEASUREMENT;
                m_rtkNisRejectTimestampUs = micros();
            }
            return;
        }

        innovation.segment<3>(3) = z_vel - m_x.segment<3>(3);
        H_rtk.block<3,3>(3,3) = Eigen::Matrix3f::Identity();
    }

    if (!USE_RTK_VERTICAL)
    {
        innovation(2) = 0.0f;
        innovation(5) = 0.0f;
        H_rtk.row(2).setZero();
        H_rtk.row(5).setZero();
    }

    float sigma_pos_horizontal = SIGMA_RTK_UNKNOWN_POS;
    float sigma_pos_vertical = SIGMA_RTK_UNKNOWN_HEIGHT;
    float sigma_vel = SIGMA_RTK_UNKNOWN_VEL;
    switch (rtk.fix_quality)
    {
        case 1:
            sigma_pos_horizontal = SIGMA_RTK_GPS_POS;
            sigma_pos_vertical = SIGMA_RTK_GPS_HEIGHT;
            sigma_vel = SIGMA_RTK_GPS_VEL;
            break;
        case 2:
            sigma_pos_horizontal = SIGMA_RTK_DGPS_POS;
            sigma_pos_vertical = SIGMA_RTK_DGPS_HEIGHT;
            sigma_vel = SIGMA_RTK_DGPS_VEL;
            break;
        case 4:
            sigma_pos_horizontal = SIGMA_RTK_FIXED_POS;
            sigma_pos_vertical = SIGMA_RTK_FIXED_HEIGHT;
            sigma_vel = SIGMA_RTK_FIXED_VEL;
            break;
        case 5:
            sigma_pos_horizontal = SIGMA_RTK_FLOAT_POS;
            sigma_pos_vertical = SIGMA_RTK_FLOAT_HEIGHT;
            sigma_vel = SIGMA_RTK_FLOAT_VEL;
            break;
        default:
            break;
    }

    Mat6 R_rtk = Mat6::Zero();
    R_rtk(0,0) = sigma_pos_horizontal * sigma_pos_horizontal;
    R_rtk(1,1) = sigma_pos_horizontal * sigma_pos_horizontal;
    R_rtk(2,2) = sigma_pos_vertical * sigma_pos_vertical;
    R_rtk.block<3,3>(3,3) = (sigma_vel * sigma_vel) * Eigen::Matrix3f::Identity();

    // Publish the pre-correction prediction and residual even when the gate
    // rejects the measurement.
    m_h.segment<3>(8) = m_x.segment<3>(0);
    m_h.segment<3>(11) = m_x.segment<3>(3);
    m_y.segment<3>(8) = innovation.segment<3>(0);
    m_y.segment<3>(11) = innovation.segment<3>(3);

    const Mat6 S = H_rtk * m_P * H_rtk.transpose() + R_rtk;
    const Eigen::LDLT<Mat6> S_ldlt(S);
    float rtk_nis = std::numeric_limits<float>::quiet_NaN();
    const bool nis_valid = calculateNis(S_ldlt, innovation, rtk_nis);
    const uint8_t position_dof = USE_RTK_VERTICAL ? 3U : 2U;
    const uint8_t active_measurement_dof = static_cast<uint8_t>(
        position_dof * (USE_RTK_VELOCITY ? 2U : 1U));
    const bool innovation_accepted = nis_valid &&
                                     rtk_nis <= nisGateThreshold(active_measurement_dof);
    if (publish_nis)
    {
        m_rtkNis = rtk_nis;
        ++m_rtkNisCount;
        m_rtkNisTimestampUs = micros();
        m_rtkNisRejectReason = innovation_accepted
            ? SensorStructs::NisRejectReason::NONE
            : (nis_valid
                ? SensorStructs::NisRejectReason::INNOVATION_GATE
                : SensorStructs::NisRejectReason::NUMERIC_FAILURE);
        m_rtkNisRejectTimestampUs = innovation_accepted ? 0 : m_rtkNisTimestampUs;
    }
    if (!innovation_accepted)
    {
        return;
    }
    Eigen::Matrix<float, 16, 6> K_rtk = m_P * H_rtk.transpose() * S_ldlt.solve(Mat6::Identity());

    if (!USE_RTK_VERTICAL)
    {
        K_rtk.row(2).setZero();
        K_rtk.row(5).setZero();
    }

    m_x += K_rtk * innovation;
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

    m_IKH.noalias()    = Eigen::Matrix<float,16,16>::Identity() - K_rtk * H_rtk;
    m_P_temp.noalias() = m_IKH * m_P;
    m_P.noalias()      = m_P_temp * m_IKH.transpose();
    m_P_temp.noalias() = K_rtk * R_rtk * K_rtk.transpose();
    m_P               += m_P_temp;
    m_P_temp           = m_P + m_P.transpose();
    m_P                = 0.5f * m_P_temp;
}

void EKF::updateLidar(const SensorStructs::LIDAR_t& lidar)
{
    const bool publish_nis = isNewMeasurement(lidar.timestamp_us,
                                               m_lastLidarNisMeasurementTime);
    if (publish_nis)
    {
        m_lastLidarNisMeasurementTime = lidar.timestamp_us;
    }

    if constexpr (!TimingConfig::EKF::LIDAR_CORRECTION_ENABLED)
    {
        if (publish_nis)
        {
            m_lidarNisRejectReason = SensorStructs::NisRejectReason::NIS_DISABLED;
            m_lidarNisRejectTimestampUs = 0;
        }
        return;
    }

    // Skip if no home reference set, measurement invalid, or out of rated range
    const float h0 = m_setHome_ref.launch_lidar_dist;
    if (h0 <= 0.0f)
    {
        if (publish_nis)
        {
            m_lidarNisRejectReason = SensorStructs::NisRejectReason::NO_HOME_REFERENCE;
            m_lidarNisRejectTimestampUs = micros();
        }
        return;
    }
    if (!lidar.valid)
    {
        if (publish_nis)
        {
            m_lidarNisRejectReason = SensorStructs::NisRejectReason::SENSOR_QUALITY_GATE;
            m_lidarNisRejectTimestampUs = micros();
        }
        return;
    }

    const float z_m = lidar.dist * 0.01f;   // cm → m
    if (!std::isfinite(z_m))
    {
        if (publish_nis)
        {
            m_lidarNisRejectReason = SensorStructs::NisRejectReason::INVALID_MEASUREMENT;
            m_lidarNisRejectTimestampUs = micros();
        }
        return;
    }
    if (z_m > LIDAR_MAX_RANGE)
    {
        if (publish_nis)
        {
            m_lidarNisRejectReason = SensorStructs::NisRejectReason::OUT_OF_RANGE;
            m_lidarNisRejectTimestampUs = micros();
        }
        return;
    }

    // ── Measurement model ─────────────────────────────────────────────────────
    // At home: pd=0, lidar reads h0. As rocket climbs, pd goes negative, lidar
    // distance grows: predicted = h0 - pd = h0 - m_x(2)
    m_h(14) = h0 - m_x(2);

    // ── Innovation ────────────────────────────────────────────────────────────
    m_y(14) = z_m - m_h(14);

    // ── Jacobian (1×16) — only the pd component is non-zero ──────────────────
    Eigen::Matrix<float, 1, 16> H_lidar = Eigen::Matrix<float, 1, 16>::Zero();
    H_lidar(0, 2) = -1.0f;

    // ── Measurement noise ─────────────────────────────────────────────────────
    const float R_lidar = SIGMA_LIDAR * SIGMA_LIDAR;

    // ── Kalman gain (16×1) ────────────────────────────────────────────────────
    const float S = (H_lidar * m_P * H_lidar.transpose())(0, 0) + R_lidar;
    const bool nis_valid = std::isfinite(S) && S > 0.0f && std::isfinite(m_y(14));
    const float lidar_nis = nis_valid
        ? m_y(14) * m_y(14) / S
        : std::numeric_limits<float>::quiet_NaN();
    const bool innovation_accepted = nis_valid &&
                                     std::isfinite(lidar_nis) &&
                                     lidar_nis <= nisGateThreshold(1);
    if (publish_nis)
    {
        m_lidarNis = lidar_nis;
        ++m_lidarNisCount;
        m_lidarNisTimestampUs = micros();
        m_lidarNisRejectReason = innovation_accepted
            ? SensorStructs::NisRejectReason::NONE
            : (nis_valid && std::isfinite(lidar_nis)
                ? SensorStructs::NisRejectReason::INNOVATION_GATE
                : SensorStructs::NisRejectReason::NUMERIC_FAILURE);
        m_lidarNisRejectTimestampUs = innovation_accepted ? 0 : m_lidarNisTimestampUs;
    }
    if (!innovation_accepted)
    {
        return;
    }
    const Eigen::Matrix<float, 16, 1> K_lidar = (m_P * H_lidar.transpose()) / S;

    // ── State update ──────────────────────────────────────────────────────────
    m_x += K_lidar * m_y(14);

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
    m_IKH.noalias()    = Eigen::Matrix<float,16,16>::Identity() - K_lidar * H_lidar;
    m_P_temp.noalias() = m_IKH * m_P;
    m_P.noalias()      = m_P_temp * m_IKH.transpose();
    m_P_temp.noalias() = K_lidar * R_lidar * K_lidar.transpose();
    m_P               += m_P_temp;
    m_P_temp           = m_P + m_P.transpose();
    m_P                = 0.5f * m_P_temp;
}
// this understands the orientation affects the value the lidar gives but its worse so maybe small angle approximations would work when its flying??
// void EKF::updateLidar(const SensorStructs::LIDAR_t& lidar)
// {
//     const float h0 = m_setHome_ref.launch_lidar_dist;
//     if (h0 <= 0.0f || !lidar.valid) { return; }

//     const float z_m = lidar.dist * 0.01f;   // cm → m
//     if (z_m > LIDAR_MAX_RANGE) { return; }

//     // ── Body z-axis in NED (lidar beam direction) ─────────────────────────────
//     // Third column of R_body_to_ned: d_ned = R(q) * (0,0,1)
//     // d_ned[2] = 1 - 2*(q1² + q2²)  — the NED-down component of the beam
//     Eigen::Vector4f q = m_x.segment<4>(6);
//     q.normalize();
//     const float q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);

//     const float cos_theta = 1.0f - 2.0f * (q1*q1 + q2*q2);

//     // Gate: skip update if tilted more than ~60° from vertical (cos < 0.5)
//     if (cos_theta < 0.5f) { return; }

//     // ── Measurement model ─────────────────────────────────────────────────────
//     // Ground is flat. The beam travels (h0 - pd) / cos_theta to reach it,
//     // where h0 - pd is the vertical height above the launch-site ground plane.
//     const float h_above = h0 - m_x(2);   // vertical height above ground (m)
//     m_h(14) = h_above / cos_theta;

//     // ── Innovation ────────────────────────────────────────────────────────────
//     m_y(14) = z_m - m_h(14);

//     // ── Jacobian (1×16) ───────────────────────────────────────────────────────
//     // d(h_pred)/d(pd) = -1/cos_theta
//     // d(h_pred)/d(q1) = h_above * 4*q1 / cos_theta²   [q1 at state index 7]
//     // d(h_pred)/d(q2) = h_above * 4*q2 / cos_theta²   [q2 at state index 8]
//     // q0 and q3 do not appear in cos_theta → zero
//     Eigen::Matrix<float, 1, 16> H_lidar = Eigen::Matrix<float, 1, 16>::Zero();
//     H_lidar(0, 2) = -1.0f / cos_theta;
//     const float cos_theta2 = cos_theta * cos_theta;
//     H_lidar(0, 7) = 4.0f * q1 * h_above / cos_theta2;
//     H_lidar(0, 8) = 4.0f * q2 * h_above / cos_theta2;

//     // ── Measurement noise ─────────────────────────────────────────────────────
//     const float R_lidar = SIGMA_LIDAR * SIGMA_LIDAR;

//     // ── Kalman gain (16×1) ────────────────────────────────────────────────────
//     const float S = (H_lidar * m_P * H_lidar.transpose())(0, 0) + R_lidar;
//     const Eigen::Matrix<float, 16, 1> K_lidar = (m_P * H_lidar.transpose()) / S;

//     // ── State update ──────────────────────────────────────────────────────────
//     m_x += K_lidar * m_y(14);

//     // ── Renormalise quaternion ────────────────────────────────────────────────
//     Eigen::Vector4f q_new = m_x.segment<4>(6);
//     const float q_norm = q_new.norm();
//     if (!std::isfinite(q_norm) || q_norm < 1e-9f)
//         m_x.segment<4>(6) = Eigen::Vector4f(1.0f, 0.0f, 0.0f, 0.0f);
//     else
//     {
//         q_new /= q_norm;
//         if (q_new(0) < 0) q_new = -q_new;
//         m_x.segment<4>(6) = q_new;
//     }

//     // ── Joseph form covariance update ─────────────────────────────────────────
//     m_IKH.noalias()    = Eigen::Matrix<float,16,16>::Identity() - K_lidar * H_lidar;
//     m_P_temp.noalias() = m_IKH * m_P;
//     m_P.noalias()      = m_P_temp * m_IKH.transpose();
//     m_P_temp.noalias() = K_lidar * R_lidar * K_lidar.transpose();
//     m_P               += m_P_temp;
//     m_P_temp           = m_P + m_P.transpose();
//     m_P                = 0.5f * m_P_temp;
// }
