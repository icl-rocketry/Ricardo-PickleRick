#include "Estimator/ekf.h"
#include "Config/timing_config.h"

namespace
{
    bool timeAtOrAfter(const uint32_t lhs, const uint32_t rhs)
    {
        return static_cast<int32_t>(lhs - rhs) >= 0;
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
}

void EKF::setup(const Eigen::Vector3f& gyro_bias, 
                const Eigen::Vector3f& accel_bias, 
                const Eigen::Vector3f& h_accel_bias,
                const Eigen::Vector3f& mag_ref
)
{
    m_x.setZero();

    // Initialise quaternion to identity [1, 0, 0, 0], rotated 90deg to the rocket orientation 
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
    m_covariancePredictDt = 0.0f;
    m_nextCorrectionIndex = 0;
    m_gnssTimeOffsetValid = false;
    m_gnssToLocalOffsetUs = 0;
    m_lastRtkDelayUs = 0;
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

    if (propagate_covariance)
    {
        return;
    }

    const bool replayedToNow = handleRtkCorrection(now, rtk);
    if (!replayedToNow)
    {
        runScheduledCorrection(now, accel, mag, baro, gps, lidar, rtk, false);
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
        F_qq =  std::cos(0.5f * w_norm * nominal_dt) * Mat4::Identity()
              + std::sin(0.5f * w_norm * nominal_dt) * (Omega / w_norm);
    }
    else // to avoid the /0
    {
        F_qq = Mat4::Identity() + 0.5f * nominal_dt * Omega;
    }
        
    m_x.segment<4>(6) = F_qq * q;
    m_x.segment<4>(6).normalize();

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
    // F is the Jacobian of the process model 
    m_F.setZero();
    m_F.block<3,3>(0,0)   = I3;
    m_F.block<3,3>(0,3)   = dt * I3;       // position depends on velocity
    m_F.block<3,3>(3,3)   = I3;            // velocity integrates
    m_F.block<4,4>(6,6)   = F_qq_cov;      // attitude
    m_F.block<3,3>(10,10) = I3;            // accel bias
    m_F.block<3,3>(13,13) = I3;            // gyro bias
    m_F.block<4,3>(6,13)  = -G_w;          // gyro bias cross term
    m_F.block<3,3>(3,10)  = -R_body_to_ned * dt;
    m_F.block<3,3>(0,10)  = -0.5f * R_body_to_ned * dt2;
    // Q is the process noise covariance — how much we trust the process model (vs measurements)
    m_Q.setZero();
    m_Q.block<6,6>(0,0)   = m_Q_trans;
    m_Q.block<4,4>(6,6)   = m_Q_att;
    m_Q.block<3,3>(10,10) = (SIGMA_BA_LOW.array().square().matrix().asDiagonal()) * dt;
    m_Q.block<3,3>(13,13) = (SIGMA_BG.array().square().matrix().asDiagonal()) * dt;

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
    m_nextCorrectionIndex = state.nextCorrectionIndex;
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

bool EKF::handleRtkCorrection(const uint32_t now, const SensorStructs::RTK_t& rtk)
{
    if (!rtk.valid ||
        rtk.fix_quality == 0 ||
        rtk.timestamp_us == 0 ||
        rtk.timestamp_us == m_lastRtkMeasurementTime)
    {
        return false;
    }

    const bool packet_fresh = now - rtk.timestamp_us <= TimingConfig::EKF::RTK_CORRECTION_MAX_AGE_US;
    if (!packet_fresh)
    {
        m_lastRtkMeasurementTime = rtk.timestamp_us;
        return false;
    }

    if (m_lastRtkCorrectionTime != 0 &&
        now - m_lastRtkCorrectionTime < TimingConfig::EKF::RTK_CORRECTION_DELTA_US)
    {
        return false;
    }

    uint32_t measurement_us = 0;
    if (gnssTimeOfDayToLocalUs(rtk.gnss_time_of_day_ms, now, measurement_us))
    {
        m_lastRtkDelayUs = timeAtOrAfter(now, measurement_us) ? now - measurement_us : 0;
    }
    else
    {
        m_lastRtkDelayUs = 0;
    }

    const bool replayed_to_now = fuseDelayedRTK(rtk, now);
    if (!replayed_to_now)
    {
        updateRTK(rtk);
    }

    m_lastRtkCorrectionTime = now;
    m_lastRtkMeasurementTime = rtk.timestamp_us;
    return replayed_to_now;
}

bool EKF::fuseDelayedRTK(const SensorStructs::RTK_t& rtk, const uint32_t now)
{
    uint32_t measurement_us = 0;
    if (!gnssTimeOfDayToLocalUs(rtk.gnss_time_of_day_ms, now, measurement_us))
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

    const HistorySample& base = m_history[static_cast<size_t>(base_index)];
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
    }

    int index = base_index;
    while (index != latest_index)
    {
        const int next_index = nextHistoryIndex(index);
        if (next_index < 0)
        {
            break;
        }

        const HistorySample& sample = m_history[static_cast<size_t>(next_index)];
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

        if (!sample.propagate_covariance)
        {
            runScheduledCorrection(sample.timestamp_us,
                                   sample.accel,
                                   sample.mag,
                                   sample.baro,
                                   sample.gps,
                                   sample.lidar,
                                   sample.rtk,
                                   false);
        }

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
                    updateLowGAccel(accel);
                    return;
                }
                break;
            case 1:
                if (mag.timestamp_us != 0 &&
                    mag.timestamp_us != m_lastMagMeasurementTime &&
                    timerDue(now, m_lastMagCorrectionTime, TimingConfig::EKF::MAG_CORRECTION_DELTA_US))
                {
                    updateMag(Eigen::Vector3f(mag.mx, mag.my, mag.mz));
                    m_lastMagMeasurementTime = mag.timestamp_us;
                    return;
                }
                break;
            case 2:
                if (baro.timestamp_us != 0 &&
                    baro.timestamp_us != m_lastBaroMeasurementTime &&
                    timerDue(now, m_lastBaroCorrectionTime, TimingConfig::EKF::BARO_CORRECTION_DELTA_US))
                {
                    updateBaro(baro.press, baro.temp);
                    m_lastBaroMeasurementTime = baro.timestamp_us;
                    return;
                }
                break;
            case 3:
                if (gps.timestamp_us != 0 &&
                    gps.timestamp_us != m_lastGpsMeasurementTime &&
                    timerDue(now, m_lastGpsCorrectionTime, TimingConfig::EKF::GPS_CORRECTION_DELTA_US))
                {
                    updateGPS(gps);
                    m_lastGpsMeasurementTime = gps.timestamp_us;
                    return;
                }
                break;
            case 4:
                if (allow_rtk &&
                    rtk.valid &&
                    rtk.fix_quality != 0 &&
                    rtk.timestamp_us != 0 &&
                    rtk.timestamp_us != m_lastRtkMeasurementTime)
                {
                    const bool fresh = now - rtk.timestamp_us <= TimingConfig::EKF::RTK_CORRECTION_MAX_AGE_US;
                    if (!fresh)
                    {
                        m_lastRtkMeasurementTime = rtk.timestamp_us;
                        break;
                    }

                    if (correctionDueNow(now, m_lastRtkCorrectionTime, TimingConfig::EKF::RTK_CORRECTION_DELTA_US))
                    {
                        updateRTK(rtk);
                        m_lastRtkMeasurementTime = rtk.timestamp_us;
                        return;
                    }
                }
                break;
            case 5:
                if (lidar.timestamp_us != 0 &&
                    lidar.timestamp_us != m_lastLidarMeasurementTime &&
                    timerDue(now, m_lastLidarCorrectionTime, TimingConfig::EKF::LIDAR_CORRECTION_DELTA_US))
                {
                    updateLidar(lidar);
                    m_lastLidarMeasurementTime = lidar.timestamp_us;
                    return;
                }
                break;
            default:
                break;
        }
    }
}

void EKF::updateLowGAccel(const Eigen::Vector3f& z_accel)
{
    using Mat3 = Eigen::Matrix3f;
    using Vec3 = Eigen::Vector3f;
    using Vec4 = Eigen::Vector4f;

    const float z_norm = z_accel.norm();
    if (!std::isfinite(z_norm) || z_norm < 1e-6f) {
        return;
    }

    const float accel_error = std::abs(z_norm - g);

    // Hard reject only when accel is clearly not gravity-dominated
    if (accel_error > ACCEL_GATE) {
        return;
    }
    // Smoothly reduce accel trust as |a| moves away from 1g
    const float scale = 1.0f + 3.0f * (accel_error / ACCEL_GATE);

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

    const Mat3 R_base = SIGMA_ACCEL_LOW.cwiseProduct(SIGMA_ACCEL_LOW).asDiagonal(); 
    const Mat3 R = scale * R_base; //here is where we reduce the trust in accel as it moves away from 1g
    
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
    if (!gps.valid || gps.fix < 1 || gps.sat < 4) { return; }

    const bool pos_valid = USE_GPS_POSITION &&
                           std::isfinite(gps.hAcc) &&
                           gps.hAcc > 0.0f &&
                           gps.hAcc <= 3.0f;
    const bool vel_valid = std::isfinite(gps.v_n) &&
                           std::isfinite(gps.v_e) &&
                           std::isfinite(gps.v_d);

    if (!pos_valid && !vel_valid) { return; }

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

void EKF::updateRTK(const SensorStructs::RTK_t& rtk)
{
    using Mat6 = Eigen::Matrix<float, 6, 6>;
    using Vec6 = Eigen::Matrix<float, 6, 1>;

    if (!rtk.valid || rtk.fix_quality == 0) { return; }

    const Eigen::Vector3f z_pos(rtk.x, rtk.y, rtk.z);
    if (!z_pos.allFinite()) { return; }

    Vec6 innovation = Vec6::Zero();
    innovation.segment<3>(0) = z_pos - m_x.segment<3>(0);

    Eigen::Matrix<float, 6, 16> H_rtk = Eigen::Matrix<float, 6, 16>::Zero();
    H_rtk.block<3,3>(0,0) = Eigen::Matrix3f::Identity();

    if (USE_RTK_VELOCITY)
    {
        const Eigen::Vector3f z_vel(rtk.u, rtk.v, rtk.w);
        if (!z_vel.allFinite()) { return; }

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

    float sigma_pos = SIGMA_RTK_UNKNOWN_POS;
    float sigma_vel = SIGMA_RTK_UNKNOWN_VEL;
    switch (rtk.fix_quality)
    {
        case 1:
            sigma_pos = SIGMA_RTK_GPS_POS;
            sigma_vel = SIGMA_RTK_GPS_VEL;
            break;
        case 2:
            sigma_pos = SIGMA_RTK_DGPS_POS;
            sigma_vel = SIGMA_RTK_DGPS_VEL;
            break;
        case 4:
            sigma_pos = SIGMA_RTK_FIXED_POS;
            sigma_vel = SIGMA_RTK_FIXED_VEL;
            break;
        case 5:
            sigma_pos = SIGMA_RTK_FLOAT_POS;
            sigma_vel = SIGMA_RTK_FLOAT_VEL;
            break;
        default:
            break;
    }

    Mat6 R_rtk = Mat6::Zero();
    R_rtk.block<3,3>(0,0) = (sigma_pos * sigma_pos) * Eigen::Matrix3f::Identity();
    R_rtk.block<3,3>(3,3) = (sigma_vel * sigma_vel) * Eigen::Matrix3f::Identity();

    const Mat6 S = H_rtk * m_P * H_rtk.transpose() + R_rtk;
    Eigen::Matrix<float, 16, 6> K_rtk = m_P * H_rtk.transpose() * S.ldlt().solve(Mat6::Identity());

    if (!USE_RTK_VERTICAL)
    {
        K_rtk.row(2).setZero();
        K_rtk.row(5).setZero();
    }

    m_x += K_rtk * innovation;
    m_h.segment<3>(8) = m_x.segment<3>(0);
    m_h.segment<3>(11) = m_x.segment<3>(3);
    m_y.segment<3>(8) = innovation.segment<3>(0);
    m_y.segment<3>(11) = innovation.segment<3>(3);

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
    // Skip if no home reference set, measurement invalid, or out of rated range
    const float h0 = m_setHome_ref.launch_lidar_dist;
    if (h0 <= 0.0f || !lidar.valid) { return; }

    const float z_m = lidar.dist * 0.01f;   // cm → m
    if (z_m > LIDAR_MAX_RANGE) { return; }

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
