#include "Estimator/estimator.h"
#include "Config/debug_config.h"

Estimator::Estimator(Types::CoreTypes::SystemStatus_t &systemstatus)
    : m_systemstatus(systemstatus),
      m_homeSet(false),
      m_settingHome(false),
      m_calibrating(false),
      m_autoHomePending(false),
      m_autoHomeTriggered(false),
      m_gpsLockStartTimeUs(0),
      m_refOrientation(1.0, 0.0, 0.0, 0.0),
      m_ekf(),
      m_calibrator(),
      rtk()
      {};

void Estimator::setup()
{
    m_accel_lpf_x.setup(TimingConfig::Estimator::FILTER_SAMPLE_RATE_HZ, TimingConfig::Estimator::ACCEL_CUTOFF_HZ); 
    m_accel_lpf_y.setup(TimingConfig::Estimator::FILTER_SAMPLE_RATE_HZ, TimingConfig::Estimator::ACCEL_CUTOFF_HZ);
    m_accel_lpf_z.setup(TimingConfig::Estimator::FILTER_SAMPLE_RATE_HZ, TimingConfig::Estimator::ACCEL_CUTOFF_HZ);

    m_gyro_lpf_x.setup(TimingConfig::Estimator::FILTER_SAMPLE_RATE_HZ, TimingConfig::Estimator::GYRO_CUTOFF_HZ);
    m_gyro_lpf_y.setup(TimingConfig::Estimator::FILTER_SAMPLE_RATE_HZ, TimingConfig::Estimator::GYRO_CUTOFF_HZ);
    m_gyro_lpf_z.setup(TimingConfig::Estimator::FILTER_SAMPLE_RATE_HZ, TimingConfig::Estimator::GYRO_CUTOFF_HZ);

    m_calibrating = false;
    m_settingHome = false;
    m_autoHomePending = false;
    m_autoHomeTriggered = false;
    m_gpsLockStartTimeUs = 0;
    m_lastGpsDebugTimestampUs = 0;
    m_rtkDelayUs = 0;
    m_calibrator.setup();
    m_ekf.setup(
        m_calibrator.getGyroBiases(),
        m_calibrator.getAccelBiases(),
        m_calibrator.getHighGBiases(),
        m_calibrator.getMagRef()
    );
    rtk.setup();
};

void Estimator::update(const SensorStructs::raw_measurements_t &raw_sensors)
{
    updateAutoHome(raw_sensors.gps);

    if (m_calibrating) {
        if (m_calibrator.getNumberOfCalibrationMeasurements() >= TimingConfig::Estimator::CALIBRATION_SAMPLE_COUNT) {
            m_calibrator.computeCalibration();
            m_ekf.setup(
                m_calibrator.getGyroBiases(),
                m_calibrator.getAccelBiases(),
                m_calibrator.getHighGBiases(),
                m_calibrator.getMagRef()
            );
            m_calibrating = false;
        } else {
            m_calibrator.updateCalibration(raw_sensors);
        }
    } else if (m_settingHome) {
        if (m_calibrator.getNumberOfSetHomeMeasurements() >= TimingConfig::Estimator::SET_HOME_SAMPLE_COUNT) {
            m_calibrator.computeSetHome();
            m_ekf.setHome(
                m_calibrator.getSetHomeRef()
            );
            if (rtk.hasFix()) {
                rtk.setHome(rtk.getPositionRaw());
            }
            m_settingHome = false;
            m_homeSet = true;
        } else {
            m_calibrator.updateSetHome(raw_sensors);
        }
    } else {
        // Raw low-g accel + gyro from IMU
        const Eigen::Vector3f gyro_raw(
            raw_sensors.accelgyro.gx,
            raw_sensors.accelgyro.gy,
            raw_sensors.accelgyro.gz
        );

        const Eigen::Vector3f accel_raw(
            raw_sensors.accelgyro.ax,
            raw_sensors.accelgyro.ay,
            raw_sensors.accelgyro.az
        );

        // Raw high-g accel
        const Eigen::Vector3f high_g_raw(
            raw_sensors.accel.ax,
            raw_sensors.accel.ay,
            raw_sensors.accel.az
        );

        // Filter low-g accel
        const Eigen::Vector3f accel_filt(
            m_accel_lpf_x.update(accel_raw.x()),
            m_accel_lpf_y.update(accel_raw.y()),
            m_accel_lpf_z.update(accel_raw.z())
        );

        // Filter gyro
        const Eigen::Vector3f gyro_filt(
            m_gyro_lpf_x.update(gyro_raw.x()),
            m_gyro_lpf_y.update(gyro_raw.y()),
            m_gyro_lpf_z.update(gyro_raw.z())
        );

        // Save raw + filtered data to telemetry/state, even though sensors should have this data its easier to have it all in the same timestep for debugging
        m_state.rawAccel = accel_raw;
        m_state.rawGyro = gyro_raw;
        m_state.filteredAccel = accel_filt;
        m_state.filteredGyro = gyro_filt;

        SensorStructs::RTK_t rtk_measurement;
        rtk.update(rtk_measurement);
        m_state.rtkUtcTimeOfDayMs = rtk_measurement.gnss_time_of_day_ms;

        // Feed filtered low-g accel + gyro into EKF
        const uint32_t ekf_start_us = micros();
        if constexpr (DebugConfig::GpsLatencyPrintEnabled)
        {
            if (raw_sensors.gps.updated &&
                raw_sensors.gps.timestamp_us != 0 &&
                raw_sensors.gps.timestamp_us != m_lastGpsDebugTimestampUs)
            {
                const uint32_t gps_delay_us = ekf_start_us - raw_sensors.gps.timestamp_us;
                const uint32_t gps_delay_ms_x10 =
                    static_cast<int32_t>(gps_delay_us) >= 0
                        ? static_cast<uint32_t>((static_cast<uint64_t>(gps_delay_us) * 10ULL + 500ULL) / 1000ULL)
                        : 0;
                Serial.printf(
                    "GPS DEBUG delay_ms=%lu.%lu solution_epoch_us=%lu estimator_parse_us=%lu iTOW_ms=%lu pps=%u pps_valid=%u\n",
                    static_cast<unsigned long>(gps_delay_ms_x10 / 10UL),
                    static_cast<unsigned long>(gps_delay_ms_x10 % 10UL),
                    static_cast<unsigned long>(raw_sensors.gps.timestamp_us),
                    static_cast<unsigned long>(ekf_start_us),
                    static_cast<unsigned long>(raw_sensors.gps.gnss_time_of_day_ms),
                    raw_sensors.gps.timestamp_from_pps ? 1U : 0U,
                    raw_sensors.gps.pps_valid ? 1U : 0U
                );
                m_lastGpsDebugTimestampUs = raw_sensors.gps.timestamp_us;
            }
        }
        SensorStructs::RTK_t rtk_timing = rtk_measurement;
        m_ekf.applyGnssTimestamp(rtk_timing, ekf_start_us);
        m_rtkDelayUs = rtkDelayUs(rtk_timing, ekf_start_us);
        m_ekf.update(
            gyro_filt,
            accel_filt,
            high_g_raw,
            raw_sensors.mag,
            raw_sensors.baro,
            raw_sensors.gps,
            raw_sensors.lidar,
            rtk_measurement
        );
        if constexpr (DebugConfig::EkfTimingPrintEnabled)
        {
            recordEkfDebugTiming(ekf_start_us, micros() - ekf_start_us);
        }
    }
    updateState();
};

void Estimator::calibrate()
{
    m_calibrating = true;
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Calibration started");

};

void Estimator::setHome()
{
    m_settingHome = true;
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Set Home started");

};

std::function<void(packetptr_t)> Estimator::registerRTK()
{
    return rtk.getThisNetworkCallback();
}

void Estimator::getRTKData(SensorStructs::RTK_t& data)
{
    const uint32_t now_us = micros();
    rtk.update(data);
    m_ekf.applyGnssTimestamp(data, now_us);
    data.delay_us = rtkDelayUs(data, now_us);
    m_rtkDelayUs = data.delay_us;
}

void Estimator::updateAutoHome(const SensorStructs::GPS_t& gps)
{
    if (m_homeSet || m_settingHome || m_autoHomeTriggered) {
        return;
    }

    if (!hasGpsLock(gps) || !rtk.hasFixed()) {
        m_autoHomePending = false;
        m_gpsLockStartTimeUs = 0;
        return;
    }

    if (!m_autoHomePending) {
        m_autoHomePending = true;
        m_gpsLockStartTimeUs = gps.timestamp_us;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("GPS lock and RTK fixed detected; auto set-home pending");
        return;
    }

    if (!m_calibrating && gps.timestamp_us - m_gpsLockStartTimeUs >= TimingConfig::Estimator::AUTO_SET_HOME_DELAY_US) {
        m_autoHomeTriggered = true;
        setHome();
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Auto set-home triggered after GPS lock and RTK fixed");
    }
}

bool Estimator::hasGpsLock(const SensorStructs::GPS_t& gps) const
{
    return gps.valid &&
           gps.timestamp_us != 0 &&
           gps.fix >= 1 &&
           gps.sat >= 4 &&
           gps.hAcc <= 3.0f;
}

uint32_t Estimator::rtkDelayUs(const SensorStructs::RTK_t& data, const uint32_t now_us) const
{
    if (data.timestamp_us == 0)
    {
        return 0;
    }

    const uint32_t delay_us = now_us - data.timestamp_us;
    return static_cast<int32_t>(delay_us) >= 0 ? delay_us : 0;
}

void Estimator::recordEkfDebugTiming(const uint32_t start_us, const uint32_t runtime_us)
{
    if (m_ekfDebugReportTimeUs == 0)
    {
        m_ekfDebugReportTimeUs = start_us;
    }

    if (m_ekfDebugPrevStartUs != 0)
    {
        const uint32_t period_us = start_us - m_ekfDebugPrevStartUs;
        m_ekfDebugPeriodTimeUs += period_us;
        m_ekfDebugPeriodCount++;
        if (period_us > m_ekfDebugMaxPeriodUs)
        {
            m_ekfDebugMaxPeriodUs = period_us;
        }
    }
    m_ekfDebugPrevStartUs = start_us;

    m_ekfDebugRuntimeUs += runtime_us;
    m_ekfDebugCount++;
    if (runtime_us > m_ekfDebugMaxRuntimeUs)
    {
        m_ekfDebugMaxRuntimeUs = runtime_us;
    }

    const uint32_t window_us = start_us - m_ekfDebugReportTimeUs;
    if (window_us < TimingConfig::MICROS_PER_SECOND)
    {
        return;
    }

    // Prints EKF loop timing once per second: achieved update rate versus target,
    // average/max period between calls, average/max EKF runtime, and call count.
    const uint32_t rate_x10 = static_cast<uint32_t>(
        (static_cast<uint64_t>(m_ekfDebugCount) * TimingConfig::MICROS_PER_SECOND * 10ULL + (window_us / 2ULL)) /
        window_us
    );
    const uint32_t avg_period_us = m_ekfDebugPeriodCount
        ? static_cast<uint32_t>(m_ekfDebugPeriodTimeUs / m_ekfDebugPeriodCount)
        : 0;
    const uint32_t avg_runtime_us = m_ekfDebugCount
        ? static_cast<uint32_t>(m_ekfDebugRuntimeUs / m_ekfDebugCount)
        : 0;
    if constexpr (DebugConfig::EkfTimingPrintEnabled)
    {
        Serial.printf(
            "EKF DEBUG rate=%lu.%luHz target=%luHz period_avg/max=%lu/%luus runtime_avg/max=%lu/%luus calls=%lu\n",
            static_cast<unsigned long>(rate_x10 / 10UL),
            static_cast<unsigned long>(rate_x10 % 10UL),
            static_cast<unsigned long>(TimingConfig::Scheduler::ESTIMATOR_UPDATE_RATE_HZ),
            static_cast<unsigned long>(avg_period_us),
            static_cast<unsigned long>(m_ekfDebugMaxPeriodUs),
            static_cast<unsigned long>(avg_runtime_us),
            static_cast<unsigned long>(m_ekfDebugMaxRuntimeUs),
            static_cast<unsigned long>(m_ekfDebugCount)
        );
    }

    m_ekfDebugReportTimeUs = start_us;
    m_ekfDebugCount = 0;
    m_ekfDebugPeriodCount = 0;
    m_ekfDebugPeriodTimeUs = 0;
    m_ekfDebugRuntimeUs = 0;
    m_ekfDebugMaxPeriodUs = 0;
    m_ekfDebugMaxRuntimeUs = 0;
}

void Estimator::updateState()
{
    // ── Orientation ───────────────────────────────────────────────────────────
    const Eigen::Vector4f q         = m_ekf.quaternion();
    m_state.orientation             = Eigen::Quaternionf(q(0), q(1), q(2), q(3));
    m_state.angularRates            = m_ekf.angularRates();
    float w = q(0), x = q(1), y = q(2), z = q(3);
    float roll  = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
    float pitch = asin(2*(w*y - z*x));
    float yaw   = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
    m_state.eulerAngles = Eigen::Vector3f(roll, pitch, yaw);

    m_state.rocketOrientation = Eigen::Quaternionf(0.70710678f, 0.0f, -0.70710678f, 0.0f) * m_state.orientation;

    const Eigen::Quaternionf rq     = m_state.rocketOrientation;
    float rw = rq.w(), rx = rq.x(), ry = rq.y(), rz = rq.z();
    float rocket_roll  = atan2(2*(rw*rx + ry*rz), 1 - 2*(rx*rx + ry*ry));
    float rocket_pitch = asin(2*(rw*ry - rz*rx));
    float rocket_yaw   = atan2(2*(rw*rz + rx*ry), 1 - 2*(ry*ry + rz*rz));
    m_state.rocketEulerAngles = Eigen::Vector3f(rocket_roll, rocket_pitch, rocket_yaw);


    // ── Navigation states ─────────────────────────────────────────────────────
    m_state.position                = m_ekf.position();
    m_state.velocity                = m_ekf.velocity();
    m_state.acceleration            = m_ekf.acceleration();
    m_state.gpsPosition             = m_ekf.gpsPosition(); 
    m_state.rtkDelayUs              = m_rtkDelayUs;

    // ── Expected Readings ─────────────────────────────────────────────────────
    m_state.expectedMagReading      = m_ekf.expectedMagReading();
    m_state.expectedAccelReading    = m_ekf.expectedAccelReading();
    m_state.expectedBaroReading     = m_ekf.expectedBaroReading();
    m_state.expectedGpsPosReading   = m_ekf.expectedGpsPosReading();
    m_state.expectedGpsVelReading   = m_ekf.expectedGpsVelReading();

    // ── Innovation  ───────────────────────────────────────────────────────────
    m_state.magInnovation           = m_ekf.magInnovation();
    m_state.accelInnovation         = m_ekf.accelInnovation();
    m_state.baroInnovation          = m_ekf.baroInnovation();
    m_state.gpsPosInnovation        = m_ekf.gpsPosInnovation();
    m_state.gpsVelInnovation        = m_ekf.gpsVelInnovation();
    m_state.expectedLidarReading    = m_ekf.expectedLidarReading();
    m_state.lidarInnovation         = m_ekf.lidarInnovation();
    m_state.magNis                  = m_ekf.magNis();
    m_state.accelNis                = m_ekf.accelNis();
    m_state.baroNis                 = m_ekf.baroNis();
    m_state.gpsNis                  = m_ekf.gpsNis();
    m_state.rtkNis                  = m_ekf.rtkNis();
    m_state.lidarNis                = m_ekf.lidarNis();
    m_state.covarianceDiagonal      = m_ekf.covarianceDiagonal();

    // ── Calibration ───────────────────────────────────────────────────────────
    m_state.accelBiases             = m_ekf.accelBias();
    m_state.gyroBiases              = m_ekf.gyroBias();
    m_state.calibration_quality     = m_calibrator.getCalibrationQuality();

    m_state.highGBiases             = m_calibrator.getHighGBiases();
    m_state.refMag                  = m_calibrator.getMagRef();
}
