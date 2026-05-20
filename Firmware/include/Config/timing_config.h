#pragma once

#include <cstdint>

namespace TimingConfig
{
    constexpr uint32_t MICROS_PER_SECOND = 1000000UL;
    constexpr uint32_t MILLIS_PER_SECOND = 1000UL;

    constexpr uint32_t periodUsFromHz(const uint32_t hz)
    {
        return (MICROS_PER_SECOND + (hz / 2UL)) / hz;
    }

    constexpr uint32_t periodMsFromHz(const uint32_t hz)
    {
        return (MILLIS_PER_SECOND + (hz / 2UL)) / hz;
    }

    namespace Scheduler
    {
        constexpr uint32_t TELEMETRY_LOG_RATE_HZ = 100;
        constexpr uint32_t ESTIMATOR_UPDATE_RATE_HZ = 120;
        constexpr uint32_t POWER_MONITOR_UPDATE_RATE_HZ = 50;
        constexpr uint32_t ESTIMATOR_LOG_RATE_HZ = 50;

        constexpr uint32_t TELEMETRY_LOG_DELTA_US = periodUsFromHz(TELEMETRY_LOG_RATE_HZ);
        constexpr uint32_t ESTIMATOR_UPDATE_DELTA_US = periodUsFromHz(ESTIMATOR_UPDATE_RATE_HZ);
        constexpr uint32_t POWER_MONITOR_UPDATE_DELTA_US = periodUsFromHz(POWER_MONITOR_UPDATE_RATE_HZ);
        constexpr uint32_t ESTIMATOR_LOG_DELTA_US = periodUsFromHz(ESTIMATOR_LOG_RATE_HZ);
    }

    namespace Estimator
    {
        //filter settings
        constexpr uint32_t UPDATE_RATE_HZ = Scheduler::ESTIMATOR_UPDATE_RATE_HZ;
        constexpr float FILTER_SAMPLE_RATE_HZ = static_cast<float>(UPDATE_RATE_HZ);
        constexpr float ACCEL_CUTOFF_HZ = 20.0f;
        constexpr float GYRO_CUTOFF_HZ = 30.0f;

        constexpr uint32_t CALIBRATION_DURATION_S = 20;
        constexpr uint32_t SET_HOME_DURATION_S = 3;
        constexpr uint32_t AUTO_SET_HOME_DELAY_S = 15;
        constexpr uint32_t CALIBRATION_SAMPLE_COUNT = CALIBRATION_DURATION_S * UPDATE_RATE_HZ;
        constexpr uint32_t SET_HOME_SAMPLE_COUNT = SET_HOME_DURATION_S * UPDATE_RATE_HZ;
        constexpr uint32_t AUTO_SET_HOME_DELAY_US = AUTO_SET_HOME_DELAY_S * MICROS_PER_SECOND;
    }

    namespace EKF
    {
        constexpr uint32_t COVARIANCE_UPDATE_RATE_HZ = 100;
        constexpr uint32_t ACCEL_CORRECTION_RATE_HZ = 20;
        constexpr uint32_t MAG_CORRECTION_RATE_HZ = 10;
        constexpr uint32_t BARO_CORRECTION_RATE_HZ = 10;
        constexpr uint32_t GPS_CORRECTION_RATE_HZ = 10; //sensor configured to this
        constexpr uint32_t LIDAR_CORRECTION_RATE_HZ = 20;

        constexpr uint32_t COVARIANCE_UPDATE_DELTA_US = periodUsFromHz(COVARIANCE_UPDATE_RATE_HZ);
        constexpr uint32_t ACCEL_CORRECTION_DELTA_US = periodUsFromHz(ACCEL_CORRECTION_RATE_HZ);
        constexpr uint32_t MAG_CORRECTION_DELTA_US = periodUsFromHz(MAG_CORRECTION_RATE_HZ);
        constexpr uint32_t BARO_CORRECTION_DELTA_US = periodUsFromHz(BARO_CORRECTION_RATE_HZ);
        constexpr uint32_t GPS_CORRECTION_DELTA_US = periodUsFromHz(GPS_CORRECTION_RATE_HZ);
        constexpr uint32_t LIDAR_CORRECTION_DELTA_US = periodUsFromHz(LIDAR_CORRECTION_RATE_HZ);
    }

    namespace Sensors
    {
        constexpr uint32_t GPS_READ_RATE_HZ = EKF::GPS_CORRECTION_RATE_HZ * 2;// read it faster than the EKF correction to stop the buffer building up and slowing the ekf
        constexpr uint32_t BARO_READ_RATE_HZ = EKF::BARO_CORRECTION_RATE_HZ;
        constexpr uint32_t MAG_READ_RATE_HZ = EKF::MAG_CORRECTION_RATE_HZ;
        constexpr uint32_t RAIL_READ_RATE_HZ = 10; //how often the battery voltage is read
        constexpr uint32_t LIDAR_READ_RATE_HZ = EKF::LIDAR_CORRECTION_RATE_HZ;

        constexpr uint32_t GPS_READ_DELTA_US = periodUsFromHz(GPS_READ_RATE_HZ);
        constexpr uint32_t BARO_READ_DELTA_US = periodUsFromHz(BARO_READ_RATE_HZ);
        constexpr uint32_t MAG_READ_DELTA_US = periodUsFromHz(MAG_READ_RATE_HZ);
        constexpr uint32_t RAIL_READ_DELTA_US = periodUsFromHz(RAIL_READ_RATE_HZ);
        constexpr uint32_t LIDAR_READ_DELTA_US = periodUsFromHz(LIDAR_READ_RATE_HZ);
    }

    namespace Controller
    {
        constexpr uint32_t ACTUATION_RATE_HZ = 120;
        constexpr uint32_t ACTUATION_DELTA_MS = periodMsFromHz(ACTUATION_RATE_HZ);
    }

    namespace PowerMonitor
    {
        constexpr uint32_t REQUEST_PERIOD_MS = 500;
        constexpr uint32_t FRESH_TIMEOUT_MS = 1000;
    }
}
