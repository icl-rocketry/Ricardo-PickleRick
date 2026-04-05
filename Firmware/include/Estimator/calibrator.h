#pragma once

#include <Preferences.h>

#include <libriccore/riccorelogging.h>

#include "Sensors/sensorStructs.h"

class Calibrator
{
public:
    void setup();
    void update(const SensorStructs::raw_measurements_t& raw_sensors);
    void compute();

    Eigen::Vector3f getGyroBiases()    { return Eigen::Vector3f(m_gx_bias,  m_gy_bias,  m_gz_bias);  }
    Eigen::Vector3f getAccelBiases()   { return Eigen::Vector3f(m_ax_bias,  m_ay_bias,  m_az_bias);  }
    Eigen::Vector3f getHighGBiases()   { return Eigen::Vector3f(m_hax_bias, m_hay_bias, m_haz_bias); }
    Eigen::Vector3f getMagRef()        { return Eigen::Vector3f(m_mag_ref_n, m_mag_ref_e, m_mag_ref_d); }
    uint32_t getNumberOfMeasurements() { return m_number_of_measurements; };
    uint8_t  getCalibrationQuality()   { return m_calibration_quality; };
private:

    void computeMagRef(float lat_deg, float lon_deg, float alt_m);
    void saveCalibration();
    void loadCalibration();
    void backupCalibration();

    uint32_t m_number_of_measurements;
    uint32_t m_valid_gps_readings;
    uint8_t  m_calibration_quality; // 0: none, 1: ic_biases but no mag 2: mag vector
    
    static constexpr float g = 9.80665f;

    // ── Accumulators ──────────────────────────────────────────────────────────
    float m_gx_accum,    m_gy_accum,    m_gz_accum;     // gyro (rad/s)
    float m_ax_accum,    m_ay_accum,    m_az_accum;     // low-g accel (m/s²)
    float m_hax_accum,   m_hay_accum,   m_haz_accum;    // high-g accel (m/s²)
    float m_lat_accum,   m_lng_accum,   m_alt_accum;    // GPS (deg, deg, m)

    // ── Biases ────────────────────────────────────────────────────────────────
    float m_gx_bias,    m_gy_bias,    m_gz_bias;     // gyro (rad/s)
    float m_ax_bias,    m_ay_bias,    m_az_bias;     // low-g accel (m/s²)
    float m_hax_bias,   m_hay_bias,   m_haz_bias;    // high-g accel (m/s²)
    float m_lat_bias,   m_lng_bias,   m_alt_bias;    // GPS (deg, deg, m)
    float m_mag_ref_n,  m_mag_ref_e,  m_mag_ref_d;   // normalised mag ref value
};