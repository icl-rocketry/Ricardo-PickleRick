#pragma once

#include <Preferences.h>

#include <libriccore/riccorelogging.h>

#include "Sensors/sensorStructs.h"

class Calibrator
{
public:
    void setup();
    void updateCalibration(const SensorStructs::raw_measurements_t& raw_sensors);
    void computeCalibration();

    void updateSetHome(const SensorStructs::raw_measurements_t& raw_sensors);
    void computeSetHome();

    Eigen::Vector3f getGyroBiases()    { return Eigen::Vector3f(m_gx_bias,  m_gy_bias,  m_gz_bias);  }
    Eigen::Vector3f getAccelBiases()   { return Eigen::Vector3f(m_ax_bias,  m_ay_bias,  m_az_bias);  }
    Eigen::Vector3f getHighGBiases()   { return Eigen::Vector3f(m_hax_bias, m_hay_bias, m_haz_bias); }
    Eigen::Vector3f getMagRef()        { return Eigen::Vector3f(m_mag_ref_n, m_mag_ref_e, m_mag_ref_d); }
    uint8_t  getCalibrationQuality()   { return m_calibration_quality; };
    
    SensorStructs::home_ref_t getSetHomeRef()       { return m_setHome_ref; }
    uint32_t getNumberOfCalibrationMeasurements()   { return m_number_of_calibration_measurements; };
    uint32_t getNumberOfSetHomeMeasurements()       { return m_number_of_setHome_measurements; };

private:

    void computeMagRef(double lat_deg, double lon_deg, float alt_m);
    void saveCalibration();
    void loadCalibration();
    void backupCalibration();

    uint32_t m_number_of_calibration_measurements;
    uint32_t m_number_of_setHome_measurements;
    uint32_t m_valid_gps_readings;
    uint8_t  m_calibration_quality; // 0: none, 1: ic_biases but no mag 2: mag vector
    
    static constexpr float g = 9.80665f;

    // ── Accumulators ──────────────────────────────────────────────────────────
    float m_gx_accum,    m_gy_accum,    m_gz_accum;     // gyro (rad/s)
    float m_ax_accum,    m_ay_accum,    m_az_accum;     // low-g accel (m/s²)
    float m_hax_accum,   m_hay_accum,   m_haz_accum;    // high-g accel (m/s²)
    
    // ── Biases ────────────────────────────────────────────────────────────────
    float m_gx_bias,    m_gy_bias,    m_gz_bias;     // gyro (rad/s)
    float m_ax_bias,    m_ay_bias,    m_az_bias;     // low-g accel (m/s²)
    float m_hax_bias,   m_hay_bias,   m_haz_bias;    // high-g accel (m/s²)
    float m_mag_ref_n,  m_mag_ref_e,  m_mag_ref_d;   // normalised mag ref value
    
    // GPS (deg, deg, m)
    int64_t    m_lat_accum,    m_lng_accum;
    float      m_alt_accum;
    float      m_pressure_accum,   m_temperature_accum;    // barometer
    float      m_lidar_accum;                              // lidar ground dist (m)
    uint32_t   m_valid_lidar_readings;
    
    SensorStructs::home_ref_t m_setHome_ref;
};