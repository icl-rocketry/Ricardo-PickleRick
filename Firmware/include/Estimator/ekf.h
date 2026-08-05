#pragma once

#include <Eigen/Dense>
#include <ArduinoJson.h>
#include <array>
#include <cstddef>
#include <string>

#include <libriccore/riccorelogging.h>
#include <librrc/Helpers/jsonconfighelper.h>

#include "Config/timing_config.h"
#include "Config/types.h"
#include "Config/systemflags_config.h"
#include "Sensors/sensors.h"
#include "Sensors/sensorStructs.h"

// ── State vector layout (16 states) ──────────────────────────────────────────
//
//  Index  Size  Description
//  -----  ----  -----------
//   0- 2    3   position NED (m)
//   3- 5    3   velocity NED (m/s)
//   6- 9    4   attitude quaternion [q0 q1 q2 q3] (scalar first body -> NED)
//  10-12    3   low-g accel bias (m/s²)
//  13-15    3   gyro bias (rad/s)
//
// Note: high-g accel bias excluded from update but handled in calibrate.
// ─────────────────────────────────────────────────────────────────────────────

class EKF
{
public:
    EKF() {};

    void setup( const Eigen::Vector3f& gyro_bias, 
                const Eigen::Vector3f& accel_bias, 
                const Eigen::Vector3f& h_accel_bias, 
                const Eigen::Vector3f& mag_ref
            );
    void update(const Eigen::Vector3f         gyro,
                const Eigen::Vector3f         accel,
                const Eigen::Vector3f         h_accel,
                const SensorStructs::MAG_3AXIS_t& mag,
                const SensorStructs::BARO_t&  baro,
                const SensorStructs::GPS_t&   gps,
                const SensorStructs::LIDAR_t& lidar,
                const SensorStructs::RTK_t&   rtk
            );
    void setHome(const SensorStructs::home_ref_t& setHome_ref);

    // ── State accessors ───────────────────────────────────────────────────────
    Eigen::Vector3f position()      const { return m_x.segment<3>(0);  }
    Eigen::Vector3f velocity()      const { return m_x.segment<3>(3);  }
    Eigen::Vector3f acceleration()  const { return m_acceleration;  } // body frame
    Eigen::Vector4f quaternion()    const { return m_x.segment<4>(6);  }
    Eigen::Vector3f angularRates()  const { return m_angular_rates;  } // body frame

    Eigen::Vector3f accelBias()     const { return m_x.segment<3>(10); }
    Eigen::Vector3f gyroBias()      const { return m_x.segment<3>(13); }
    
    Eigen::Vector3f expectedMagReading()     const { return m_h.segment<3>(0);  }
    Eigen::Vector3f expectedAccelReading()   const { return m_h.segment<3>(3);  }
    Eigen::Vector2f expectedBaroReading()    const { return m_h.segment<2>(6);  }
    Eigen::Vector3f expectedGpsPosReading()  const { return m_h.segment<3>(8);  }
    Eigen::Vector3f expectedGpsVelReading()  const { return m_h.segment<3>(11); }
    float           expectedLidarReading()   const { return m_h(14); }

    Eigen::Vector3f magInnovation()     const { return m_y.segment<3>(0);  }
    Eigen::Vector3f accelInnovation()   const { return m_y.segment<3>(3);  }
    Eigen::Vector2f baroInnovation()    const { return m_y.segment<2>(6);  }
    Eigen::Vector3f gpsPosInnovation()  const { return m_y.segment<3>(8);  }
    Eigen::Vector3f gpsVelInnovation()  const { return m_y.segment<3>(11); }
    float           lidarInnovation()   const { return m_y(14); }
    float magNis() const { return m_magNis; }
    float accelNis() const { return m_accelNis; }
    float baroNis() const { return m_baroNis; }
    float gpsNis() const { return m_gpsNis; }
    float rtkNis() const { return m_rtkNis; }
    float lidarNis() const { return m_lidarNis; }
    uint32_t magNisCount() const { return m_magNisCount; }
    uint32_t accelNisCount() const { return m_accelNisCount; }
    uint32_t baroNisCount() const { return m_baroNisCount; }
    uint32_t gpsNisCount() const { return m_gpsNisCount; }
    uint32_t rtkNisCount() const { return m_rtkNisCount; }
    uint32_t lidarNisCount() const { return m_lidarNisCount; }
    uint32_t magNisTimestampUs() const { return m_magNisTimestampUs; }
    uint32_t accelNisTimestampUs() const { return m_accelNisTimestampUs; }
    uint32_t baroNisTimestampUs() const { return m_baroNisTimestampUs; }
    uint32_t gpsNisTimestampUs() const { return m_gpsNisTimestampUs; }
    uint32_t rtkNisTimestampUs() const { return m_rtkNisTimestampUs; }
    uint32_t lidarNisTimestampUs() const { return m_lidarNisTimestampUs; }
    uint8_t magNisRejectReason() const { return static_cast<uint8_t>(m_magNisRejectReason); }
    uint8_t accelNisRejectReason() const { return static_cast<uint8_t>(m_accelNisRejectReason); }
    uint8_t baroNisRejectReason() const { return static_cast<uint8_t>(m_baroNisRejectReason); }
    uint8_t gpsNisRejectReason() const { return static_cast<uint8_t>(m_gpsNisRejectReason); }
    uint8_t rtkNisRejectReason() const { return static_cast<uint8_t>(m_rtkNisRejectReason); }
    uint8_t lidarNisRejectReason() const { return static_cast<uint8_t>(m_lidarNisRejectReason); }
    uint32_t magNisRejectTimestampUs() const { return m_magNisRejectTimestampUs; }
    uint32_t accelNisRejectTimestampUs() const { return m_accelNisRejectTimestampUs; }
    uint32_t baroNisRejectTimestampUs() const { return m_baroNisRejectTimestampUs; }
    uint32_t gpsNisRejectTimestampUs() const { return m_gpsNisRejectTimestampUs; }
    uint32_t rtkNisRejectTimestampUs() const { return m_rtkNisRejectTimestampUs; }
    uint32_t lidarNisRejectTimestampUs() const { return m_lidarNisRejectTimestampUs; }
    Eigen::Matrix<float, 16, 1> covarianceDiagonal() const { return m_P.diagonal(); }

    Eigen::Vector3f gpsPosition()   const { return m_gps_position; }
    uint32_t rtkDelayUs() const { return m_lastRtkDelayUs; }
    bool applyGnssTimestamp(SensorStructs::RTK_t& rtk, uint32_t now) const;


private:
    uint32_t m_lastPredictTime = 0;
    uint32_t m_lastCovarianceUpdateTime = 0;
    uint32_t m_lastAccelCorrectionTime = 0;
    uint32_t m_lastMagCorrectionTime = 0;
    uint32_t m_lastBaroCorrectionTime = 0;
    uint32_t m_lastGpsCorrectionTime = 0;
    uint32_t m_lastLidarCorrectionTime = 0;
    uint32_t m_lastRtkCorrectionTime = 0;
    uint32_t m_lastMagMeasurementTime = 0;
    uint32_t m_lastBaroMeasurementTime = 0;
    uint32_t m_lastGpsMeasurementTime = 0;
    uint32_t m_lastLidarMeasurementTime = 0;
    uint32_t m_lastRtkMeasurementTime = 0;
    uint32_t m_lastRtkMeasurementEpochMs = 0;
    // These live-input watermarks are deliberately not part of the rewindable
    // schedule state. They prevent an out-of-sequence replay from making an
    // already-consumed sensor packet look new again.
    uint32_t m_lastHandledGpsTimestampUs = 0;
    uint32_t m_lastFusedGpsTimestampUs = 0;
    uint32_t m_lastHandledRtkTimestampUs = 0;
    uint32_t m_lastHandledRtkEpochMs = 0;
    uint32_t m_lastFusedRtkTimestampUs = 0;
    uint32_t m_lastAttitudeInitMagMeasurementTime = 0;
    float m_covariancePredictDt = 0.0f;
    uint8_t m_nextCorrectionIndex = 0;

    Eigen::Vector3f m_h_accel_bias{0,0,0};                  // (m/s^2) (body)
    Eigen::Vector3f m_mag_ref{1,0,0};                       // NED reference field (unit)
    SensorStructs::home_ref_t m_setHome_ref;

    Eigen::Vector3f m_acceleration;                         // body frame (m/s^2)
    Eigen::Vector3f m_angular_rates;                        // body frame (rad/s)
    
    static constexpr float g = 9.80665f;
    static constexpr float LOW_G_SATURATION  = 7.0f * g; 
    //--Acceleration gating to prevent low-g accel updates
    static constexpr float ACCEL_GATE = 0.8f;  // m/s² how much above and below of g should we accept
    static constexpr uint16_t ATTITUDE_INIT_SETTLED_SAMPLE_COUNT = 30;
    static constexpr float ATTITUDE_INIT_ACCEL_GATE = 0.6f;
    static constexpr float ATTITUDE_INIT_GYRO_MAX_RAD_S = 0.15f;

    // ── State and covariance ──────────────────────────────────────────────────
    Eigen::Matrix<float, 16, 1>   m_x;   // state vector
    Eigen::Matrix<float, 16, 16>  m_P;   // covariance matrix
    Eigen::Matrix<float, 15, 1>   m_h;   // expected sensor reading vector  (mag, accel, baro, gps_pos, gps_vel, lidar)
    Eigen::Matrix<float, 15, 1>   m_y;   // innovation vector               (mag, accel, baro, gps_pos, gps_vel, lidar)
    float m_magNis{-1.0f};
    float m_accelNis{-1.0f};
    float m_baroNis{-1.0f};
    float m_gpsNis{-1.0f};
    float m_rtkNis{-1.0f};
    float m_lidarNis{-1.0f};
    uint32_t m_magNisCount{0};
    uint32_t m_accelNisCount{0};
    uint32_t m_baroNisCount{0};
    uint32_t m_gpsNisCount{0};
    uint32_t m_rtkNisCount{0};
    uint32_t m_lidarNisCount{0};
    uint32_t m_magNisTimestampUs{0};
    uint32_t m_accelNisTimestampUs{0};
    uint32_t m_baroNisTimestampUs{0};
    uint32_t m_gpsNisTimestampUs{0};
    uint32_t m_rtkNisTimestampUs{0};
    uint32_t m_lidarNisTimestampUs{0};
    SensorStructs::NisRejectReason m_magNisRejectReason{SensorStructs::NisRejectReason::NOT_CALCULATED_YET};
    SensorStructs::NisRejectReason m_accelNisRejectReason{SensorStructs::NisRejectReason::NOT_CALCULATED_YET};
    SensorStructs::NisRejectReason m_baroNisRejectReason{SensorStructs::NisRejectReason::NOT_CALCULATED_YET};
    SensorStructs::NisRejectReason m_gpsNisRejectReason{SensorStructs::NisRejectReason::NOT_CALCULATED_YET};
    SensorStructs::NisRejectReason m_rtkNisRejectReason{SensorStructs::NisRejectReason::NOT_CALCULATED_YET};
    SensorStructs::NisRejectReason m_lidarNisRejectReason{SensorStructs::NisRejectReason::NOT_CALCULATED_YET};
    uint32_t m_magNisRejectTimestampUs{0};
    uint32_t m_accelNisRejectTimestampUs{0};
    uint32_t m_baroNisRejectTimestampUs{0};
    uint32_t m_gpsNisRejectTimestampUs{0};
    uint32_t m_rtkNisRejectTimestampUs{0};
    uint32_t m_lidarNisRejectTimestampUs{0};
    // NIS is published once per physical measurement. Historical replay may
    // recompute a correction, but must not make its diagnostics look new.
    uint32_t m_lastAccelNisMeasurementTime{0};
    uint32_t m_lastMagNisMeasurementTime{0};
    uint32_t m_lastBaroNisMeasurementTime{0};
    uint32_t m_lastGpsNisMeasurementTime{0};
    uint32_t m_lastRtkNisMeasurementTime{0};
    uint32_t m_lastLidarNisMeasurementTime{0};
    

    Eigen::Matrix<float, 16, 16>  m_F;      // state transition
    Eigen::Matrix<float, 16, 16>  m_Q;      // process noise
    Eigen::Matrix<float, 16, 3>   m_K;      // Kalman gain
    Eigen::Matrix<float, 16, 16>  m_IKH;    // (I - KH) for Joseph form

    // ── Predict intermediates ─────────────────────────────────────────────────
    Eigen::Matrix<float, 6, 6>    m_Q_trans;
    Eigen::Matrix<float, 4, 4>    m_Q_att;

    // ── Shared measurement Jacobian (reused across all update steps) ──────────
    Eigen::Matrix<float, 3, 16>   m_H;

    // ── Scratch space for matrix products — avoids 22×22 stack temporaries ───
    Eigen::Matrix<float, 16, 16>  m_P_temp;

    // ── Process noise tuning ──────────────────────────────────────────────────
    static constexpr float SIGMA_ACCEL_PROCESS = 0.5f; // m/s², white acceleration process noise
    static inline const Eigen::Vector3f SIGMA_BG {5e-5f, 5e-5f, 5e-5f};     // rad/s 
    static inline const Eigen::Vector3f SIGMA_BA_LOW{5e-4f, 5e-4f, 5e-4f};     // m/s² how much the bias can change per second (low-g accel bias)

    static inline const Eigen::Vector3f SIGMA_ALPHA{0.1f, 0.05f, 0.05f};  // rad/s
    static inline const Eigen::Vector3f SIGMA_ACCEL_LOW{0.4f, 0.7f, 1.3f};
    // static inline const Eigen::Vector3f SIGMA_ALPHA{0.01f, 0.01f, 0.01f};  // rad/s 
    // static inline const Eigen::Vector3f SIGMA_ACCEL_LOW{0.01f, 0.01f, 0.01f};
    static constexpr float SIGMA_MAG_HEADING = 0.1f;   // rad, horizontal mag heading noise

    static constexpr float SIGMA_T          = 3.0f;      // K
    static constexpr float SIGMA_P          = 12.0f;    // Pa

    static constexpr float SIGMA_GPS_VEL_HORIZONTAL = 0.3f; // m/s — north/east GPS velocity
    static constexpr float SIGMA_GPS_VEL_VERTICAL   = 0.5f; // m/s — down GPS velocity; less trusted due to vertical lag
    static constexpr float SIGMA_RTK_GPS_POS      = 2.5f;   // m horizontal, NMEA fix quality 1
    static constexpr float SIGMA_RTK_GPS_HEIGHT   = 5.0f;   // m vertical/height, NMEA fix quality 1
    static constexpr float SIGMA_RTK_GPS_VEL      = 0.5f;   // m/s
    static constexpr float SIGMA_RTK_DGPS_POS     = 0.5f;   // m horizontal, NMEA fix quality 2
    static constexpr float SIGMA_RTK_DGPS_HEIGHT  = 1.0f;   // m vertical/height, NMEA fix quality 2
    static constexpr float SIGMA_RTK_DGPS_VEL     = 0.25f;  // m/s
    static constexpr float SIGMA_RTK_FIXED_POS    = 0.05f;   // m horizontal, NMEA fix quality 4
    static constexpr float SIGMA_RTK_FIXED_HEIGHT = 0.1f;   // m vertical/height, NMEA fix quality 4
    static constexpr float SIGMA_RTK_FIXED_VEL    = 0.1f;   // m/s
    static constexpr float SIGMA_RTK_FLOAT_POS    = 0.2f;   // m horizontal, NMEA fix quality 5
    static constexpr float SIGMA_RTK_FLOAT_HEIGHT = 0.4f;   // m vertical/height, NMEA fix quality 5
    static constexpr float SIGMA_RTK_FLOAT_VEL    = 0.15f;  // m/s
    static constexpr float SIGMA_RTK_UNKNOWN_POS  = 1.0f;   // m horizontal
    static constexpr float SIGMA_RTK_UNKNOWN_HEIGHT = 2.0f; // m vertical/height
    static constexpr float SIGMA_RTK_UNKNOWN_VEL  = 0.5f;   // m/s
    static constexpr float SIGMA_LIDAR     = 0.1f;      // m — conservative, datasheet ±6cm @ 0-3m
    static constexpr float LIDAR_MAX_RANGE = 8.0f;      // m — TF-Luna rated range
    //------Measurement flags--------------------------------------------
    static constexpr bool USE_GPS_POSITION = false;
    static constexpr bool USE_GPS_VELOCITY_DIRECT = false;
    static constexpr bool USE_ACCEL_FOR_VELOCITY = true; //set to false if you just want to have the GPS velocity to be used for the vehicle velocity
    static constexpr bool USE_RTK_VERTICAL = true;
    static constexpr bool USE_RTK_VELOCITY = false;
    static constexpr uint32_t GNSS_DAY_MS = 86400UL * 1000UL;
    static constexpr uint32_t GPS_UTC_OFFSET_MS = 18000UL;
    static constexpr uint64_t GNSS_DAY_US = 86400ULL * 1000000ULL;
    static constexpr size_t HISTORY_SAMPLE_COUNT =
        (TimingConfig::EKF::DELAYED_MEASUREMENT_HISTORY_US /
         TimingConfig::Scheduler::ESTIMATOR_UPDATE_DELTA_US) + 8;

    struct CorrectionScheduleState
    {
        uint32_t lastAccelCorrectionTime{0};
        uint32_t lastMagCorrectionTime{0};
        uint32_t lastBaroCorrectionTime{0};
        uint32_t lastGpsCorrectionTime{0};
        uint32_t lastLidarCorrectionTime{0};
        uint32_t lastRtkCorrectionTime{0};
        uint32_t lastMagMeasurementTime{0};
        uint32_t lastBaroMeasurementTime{0};
        uint32_t lastGpsMeasurementTime{0};
        uint32_t lastLidarMeasurementTime{0};
        uint32_t lastRtkMeasurementTime{0};
        uint32_t lastRtkMeasurementEpochMs{0};
        uint8_t nextCorrectionIndex{0};
    };

    struct HistorySample
    {
        bool valid{false};
        uint32_t timestamp_us{0};
        float dt{0.0f};
        float covariance_dt{0.0f};
        bool propagate_covariance{false};
        Eigen::Vector3f gyro{0.0f, 0.0f, 0.0f};
        Eigen::Vector3f accel{0.0f, 0.0f, 0.0f};
        Eigen::Vector3f h_accel{0.0f, 0.0f, 0.0f};
        SensorStructs::MAG_3AXIS_t mag{};
        SensorStructs::BARO_t baro{};
        SensorStructs::GPS_t gps{};
        SensorStructs::LIDAR_t lidar{};
        SensorStructs::RTK_t rtk{};
        Eigen::Matrix<float, 16, 1> x;
        Eigen::Matrix<float, 16, 16> P;
        CorrectionScheduleState schedule;
    };

    void predict(   const float nominal_dt,
                    const float covariance_dt,
                    const bool propagate_covariance,
                    const Eigen::Vector3f gyro,
                    const Eigen::Vector3f accel,
                    const Eigen::Vector3f h_accel
                );
    void runScheduledCorrection(const uint32_t now,
                                const Eigen::Vector3f& accel,
                                const SensorStructs::MAG_3AXIS_t& mag,
                                const SensorStructs::BARO_t& baro,
                                const SensorStructs::GPS_t& gps,
                                const SensorStructs::LIDAR_t& lidar,
                                const SensorStructs::RTK_t& rtk,
                                bool allow_gps,
                                bool allow_rtk);
    bool handleGpsCorrection(uint32_t now, const SensorStructs::GPS_t& gps);
    bool fuseDelayedGPS(const SensorStructs::GPS_t& gps, uint32_t now);
    bool handleRtkCorrection(uint32_t now, const SensorStructs::RTK_t& rtk);
    bool fuseDelayedRTK(const SensorStructs::RTK_t& rtk, uint32_t now);
    void updateGnssTimeOffset(const SensorStructs::GPS_t& gps);
    bool gnssTimeOfDayToLocalUs(uint32_t gnss_time_of_day_ms, uint32_t now, uint32_t& local_us) const;
    bool initialiseAttitudeIfSettled(const Eigen::Vector3f& gyro,
                                     const Eigen::Vector3f& accel,
                                     const SensorStructs::MAG_3AXIS_t& mag);
    bool buildInitialAttitude(const Eigen::Vector3f& accel_body,
                              const Eigen::Vector3f& mag_body,
                              Eigen::Quaternionf& q_body_to_ned) const;
    void resetHistory();
    void saveHistorySample(uint32_t now,
                           float dt,
                           float covariance_dt,
                           bool propagate_covariance,
                           const Eigen::Vector3f& gyro,
                           const Eigen::Vector3f& accel,
                           const Eigen::Vector3f& h_accel,
                           const SensorStructs::MAG_3AXIS_t& mag,
                           const SensorStructs::BARO_t& baro,
                           const SensorStructs::GPS_t& gps,
                           const SensorStructs::LIDAR_t& lidar,
                           const SensorStructs::RTK_t& rtk);
    void overwriteLatestHistoryState();
    int findHistoryIndexAtOrBefore(uint32_t timestamp_us) const;
    int latestHistoryIndex() const;
    int nextHistoryIndex(int index) const;
    CorrectionScheduleState captureScheduleState() const;
    void restoreScheduleState(const CorrectionScheduleState& state);
    void updateMag(const Eigen::Vector3f& z_meas_raw, uint32_t measurement_time_us);
    void updateLowGAccel(const Eigen::Vector3f& z_accel, uint32_t measurement_time_us);
    void updateBaro(float pressure, float temperature, uint32_t measurement_time_us);
    void updateGPS(const SensorStructs::GPS_t& gps);
    void updateLidar(const SensorStructs::LIDAR_t& lidar);
    void updateRTK(const SensorStructs::RTK_t& rtk);

    
    // ── Atmospheric model constants ───────────────────────────────────────────
    static constexpr float BARO_M_0   = 0.0289644f;  // kg/mol  molar mass of air
    static constexpr float BARO_R_GAS = 8.31446f;    // J/(mol·K) universal gas constant
    static constexpr float BARO_L     = -0.0065f;    // K/m  ISA lapse rate

    // ── WGS84 constants ───────────────────────────────────────────────────────
    static constexpr double GPS_A_EARTH = 6378137.0;
    static constexpr double GPS_E2      = 0.00669437999014;

    Eigen::Vector3f m_gps_position;
    bool m_attitudeInitialised = false;
    uint16_t m_attitudeInitSampleCount = 0;
    Eigen::Vector3f m_attitudeInitAccelAccum{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f m_attitudeInitMagAccum{0.0f, 0.0f, 0.0f};
    std::array<HistorySample, HISTORY_SAMPLE_COUNT> m_history{};
    size_t m_historyHead = 0;
    size_t m_historyCount = 0;
    bool m_gnssTimeOffsetValid = false;
    int64_t m_gnssToLocalOffsetUs = 0;
    uint32_t m_lastRtkDelayUs = 0;

};
