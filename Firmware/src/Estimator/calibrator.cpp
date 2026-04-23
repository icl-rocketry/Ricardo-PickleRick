#include "Estimator/calibrator.h"

void Calibrator::setup()
{
    m_gx_accum = 0.0f;
    m_gy_accum = 0.0f;
    m_gz_accum = 0.0f;

    m_ax_accum = 0.0f;
    m_ay_accum = 0.0f;
    m_az_accum = 0.0f;

    m_lat_accum = 0;
    m_lng_accum = 0;
    m_alt_accum = 0.0f;

    m_number_of_calibration_measurements = 0;
    m_number_of_setHome_measurements = 0;
    m_valid_gps_readings = 0;

    loadCalibration();
}

void Calibrator::updateCalibration(const SensorStructs::raw_measurements_t &raw_sensors)
{
    m_gx_accum    +=    raw_sensors.accelgyro.gx;
    m_gy_accum    +=    raw_sensors.accelgyro.gy;
    m_gz_accum    +=    raw_sensors.accelgyro.gz;

    m_ax_accum    +=    raw_sensors.accelgyro.ax;
    m_ay_accum    +=    raw_sensors.accelgyro.ay;
    m_az_accum    +=    raw_sensors.accelgyro.az;

    m_hax_accum   +=    raw_sensors.accel.ax;
    m_hay_accum   +=    raw_sensors.accel.ay;
    m_haz_accum   +=    raw_sensors.accel.az;

    m_lat_accum   +=    raw_sensors.gps.latitude;
    m_lng_accum   +=    raw_sensors.gps.longitude;
    m_alt_accum   +=    raw_sensors.gps.altitude;

    if (raw_sensors.gps.valid) { m_valid_gps_readings++; }
    m_number_of_calibration_measurements++;

};

void Calibrator::computeCalibration()
{
    if (m_number_of_calibration_measurements == 0) { return; }

    const float n = static_cast<float>(m_number_of_calibration_measurements);

    m_gx_bias  = m_gx_accum  / n;
    m_gy_bias  = m_gy_accum  / n;
    m_gz_bias  = m_gz_accum  / n;

    m_ax_bias  = m_ax_accum  / n;
    m_ay_bias  = m_ay_accum  / n;
    m_az_bias  = m_az_accum  / n + g;

    m_hax_bias = m_hax_accum / n;
    m_hay_bias = m_hay_accum / n;
    m_haz_bias = m_haz_accum / n + g;

    if (m_number_of_calibration_measurements == m_valid_gps_readings) { 
        const double lat = (static_cast<double>(m_lat_accum) / n) * 1e-7;
        const double lon = (static_cast<double>(m_lng_accum) / n) * 1e-7;
        const float  alt = m_alt_accum / n;

        computeMagRef(lat, lon, alt);
        m_calibration_quality = 2;
    } else {
        computeMagRef(51.5074, -0.1278, 0.0f);
        m_calibration_quality = 1;
    }

    saveCalibration();

    m_gx_accum = 0.0f;
    m_gy_accum = 0.0f;
    m_gz_accum = 0.0f;

    m_ax_accum = 0.0f;
    m_ay_accum = 0.0f;
    m_az_accum = 0.0f;

    m_lat_accum = 0;
    m_lng_accum = 0;
    m_alt_accum = 0.0f;

    m_number_of_calibration_measurements = 0.0f;
    m_valid_gps_readings = 0.0f;

}

void Calibrator::updateSetHome(const SensorStructs::raw_measurements_t &raw_sensors)
{

    m_lat_accum   +=    raw_sensors.gps.latitude;
    m_lng_accum   +=    raw_sensors.gps.longitude;
    m_alt_accum   +=    raw_sensors.gps.altitude;

    m_pressure_accum    += raw_sensors.baro.press;
    m_temperature_accum += raw_sensors.baro.temp;

    m_number_of_setHome_measurements++;

};

void Calibrator::computeSetHome()
{
    if (m_number_of_setHome_measurements == 0) { return; }

    float n = static_cast<float>(m_number_of_setHome_measurements);

    // ── GPS reference — average in int64 then round back to int32 ────────────
    m_setHome_ref.launch_lat = static_cast<int32_t>(m_lat_accum / m_number_of_setHome_measurements);
    m_setHome_ref.launch_lon = static_cast<int32_t>(m_lng_accum / m_number_of_setHome_measurements);
    m_setHome_ref.launch_alt = m_alt_accum / n;

    // ── Baro reference ────────────────────────────────────────────────────────
    m_setHome_ref.launch_pressure    = m_pressure_accum    / n;
    m_setHome_ref.launch_temperature = m_temperature_accum / n;

    // ── Reset accumulators ────────────────────────────────────────────────────
    m_lat_accum          = 0;
    m_lng_accum          = 0;
    m_alt_accum          = 0.0f;
    m_pressure_accum     = 0.0f;
    m_temperature_accum  = 0.0f;
    m_number_of_setHome_measurements = 0;
}

void Calibrator::computeMagRef(double lat_deg, double lon_deg, float alt_m)
{
    // ── IGRF-14 degree-1 coefficients (epoch 2025.0, units: nT) ──────────────
    static constexpr double g10 = -29351.0;
    static constexpr double g11 =  -1411.0;
    static constexpr double h11 =   4766.0;

    // ── Earth radius and IGRF reference radius ────────────────────────────────
    static constexpr double Re = 6371200.0;
    static constexpr double a  = 6371200.0;

    const double r      = Re + static_cast<double>(alt_m);
    const double ratio  = a / r;
    const double ratio3 = ratio * ratio * ratio;

    // ── Convert to radians ────────────────────────────────────────────────────
    static constexpr double DEG2RAD = M_PI / 180.0;
    const double lat = lat_deg * DEG2RAD;
    const double lon = lon_deg * DEG2RAD;

    const double sinLat = std::sin(lat);
    const double cosLat = std::cos(lat);
    const double sinLon = std::sin(lon);
    const double cosLon = std::cos(lon);

    // ── Spherical harmonic field components (geocentric, nT) ─────────────────
    const double Br =  ratio3 * 2.0 * (g10 * sinLat
                                      + g11 * cosLat * cosLon
                                      + h11 * cosLat * sinLon);

    const double Bt = -ratio3 * (-g10 * cosLat
                                 + g11 * sinLat * cosLon
                                 + h11 * sinLat * sinLon);

    const double Bp = -ratio3 * (-g11 * sinLon
                                 + h11 * cosLon);

    // ── Convert geocentric spherical to NED ───────────────────────────────────
    const Eigen::Vector3d mag_ned(-Bt,   // North
                                   Bp,   // East
                                  -Br);  // Down

    // ── Normalise and store as float ──────────────────────────────────────────
    const Eigen::Vector3d mag_ref = mag_ned.normalized();
    m_mag_ref_n = static_cast<float>(mag_ref(0));
    m_mag_ref_e = static_cast<float>(mag_ref(1));
    m_mag_ref_d = static_cast<float>(mag_ref(2));
}

void Calibrator::saveCalibration()
{
    Preferences pref;
    if (!pref.begin("CAL"))
    {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(
            "Calibrator - NVS failed to start, calib not saved");
        return;
    }

    pref.putFloat("gx",  m_gx_bias);
    pref.putFloat("gy",  m_gy_bias);
    pref.putFloat("gz",  m_gz_bias);

    pref.putFloat("ax",  m_ax_bias);
    pref.putFloat("ay",  m_ay_bias);
    pref.putFloat("az",  m_az_bias);

    pref.putFloat("hax", m_hax_bias);
    pref.putFloat("hay", m_hay_bias);
    pref.putFloat("haz", m_haz_bias);

    pref.putFloat("mrn", m_mag_ref_n);
    pref.putFloat("mre", m_mag_ref_e);
    pref.putFloat("mrd", m_mag_ref_d);

    pref.putUChar("q",   m_calibration_quality);

    pref.end();

    char buf[200];
    snprintf(buf, sizeof(buf),
        "Calibrator - Calib saved q=%u "
        "gyro=[%.4f,%.4f,%.4f] "
        "accel=[%.4f,%.4f,%.4f] "
        "haccel=[%.4f,%.4f,%.4f] "
        "magref=[%.4f,%.4f,%.4f]",
        m_calibration_quality,
        m_gx_bias,   m_gy_bias,   m_gz_bias,
        m_ax_bias,   m_ay_bias,   m_az_bias,
        m_hax_bias,  m_hay_bias,  m_haz_bias,
        m_mag_ref_n, m_mag_ref_e, m_mag_ref_d);

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(buf);
    loadCalibration();
};

void Calibrator::loadCalibration()
{
    Preferences pref;
    if (!pref.begin("CAL", true))  // true = read-only
    {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(
            "Calibrator - NVS failed to start, calibration not loaded");
        backupCalibration();
        return;
    }

    m_gx_bias  =            pref.getFloat("gx",  0.0f);
    m_gy_bias  =            pref.getFloat("gy",  0.0f);
    m_gz_bias  =            pref.getFloat("gz",  0.0f);

    m_ax_bias  =            pref.getFloat("ax",  0.0f);
    m_ay_bias  =            pref.getFloat("ay",  0.0f);
    m_az_bias  =            pref.getFloat("az",  0.0f);

    m_hax_bias =            pref.getFloat("hax", 0.0f);
    m_hay_bias =            pref.getFloat("hay", 0.0f);
    m_haz_bias =            pref.getFloat("haz", 0.0f);

    m_calibration_quality = pref.getUChar("q", 0);

    m_mag_ref_n =           pref.getFloat("mrn", 0.0f);
    m_mag_ref_e =           pref.getFloat("mre", 0.0f);
    m_mag_ref_d =           pref.getFloat("mrd", 0.0f);
        
    // Sanity check the loaded ref
    const float norm = Eigen::Vector3f(m_mag_ref_n, m_mag_ref_e, m_mag_ref_d).norm();
    if (norm < 0.9f || norm > 1.1f)
    {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(
            "Calibrator - stored mag ref invalid, using London default");
        computeMagRef(51.5750f, -0.1453f, 0.0f);
        m_calibration_quality = 1;
    }

    pref.end();

    char buf[200];
    snprintf(buf, sizeof(buf),
        "Calibrator - Calib loaded q=%u "
        "gyro=[%.4f,%.4f,%.4f] "
        "accel=[%.4f,%.4f,%.4f] "
        "haccel=[%.4f,%.4f,%.4f] "
        "magref=[%.4f,%.4f,%.4f]",
        m_calibration_quality,
        m_gx_bias,   m_gy_bias,   m_gz_bias,
        m_ax_bias,   m_ay_bias,   m_az_bias,
        m_hax_bias,  m_hay_bias,  m_haz_bias,
        m_mag_ref_n, m_mag_ref_e, m_mag_ref_d);
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(buf);

};

void Calibrator::backupCalibration()
{
    m_gx_bias  = 0.0f;
    m_gy_bias  = 0.0f;
    m_gz_bias  = 0.0f;

    m_ax_bias  = 0.0f;
    m_ay_bias  = 0.0f;
    m_az_bias  = 0.0f;

    m_hax_bias = 0.0f;
    m_hay_bias = 0.0f;
    m_haz_bias = 0.0f;

    computeMagRef(51.5750f, -0.1453f, 0.0f);

    m_calibration_quality = 0;
}