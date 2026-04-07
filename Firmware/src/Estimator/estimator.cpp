#include "Estimator/estimator.h"

Estimator::Estimator(Types::CoreTypes::SystemStatus_t &systemstatus)
    : m_systemstatus(systemstatus),
      m_update_frequency(2000),  // 500Hz update
      m_homeSet(false),
      m_refOrientation(1.0, 0.0, 0.0, 0.0),
      m_ekf(),
      m_calibrator()
      {};

void Estimator::setup()
{
    m_calibrating = false;
    m_settingHome = false;
    m_calibrator.setup();
    m_ekf.setup(
        m_calibrator.getGyroBiases(),
        m_calibrator.getAccelBiases(),
        m_calibrator.getHighGBiases(),
        m_calibrator.getMagRef()
    );
};

void Estimator::update(const SensorStructs::raw_measurements_t &raw_sensors)
{
    if (m_calibrating) {
        if (m_calibrator.getNumberOfCalibrationMeasurements() >= 10 * 500) {
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
        if (m_calibrator.getNumberOfSetHomeMeasurements() >= 3 * 500) {
            m_calibrator.computeSetHome();
            m_ekf.setHome(
                m_calibrator.getSetHomeRef()
            );
            m_settingHome = false;
            m_homeSet = true;
        } else {
            m_calibrator.updateSetHome(raw_sensors);
        }
    } else {
        m_ekf.update(
            Eigen::Vector3f(raw_sensors.accelgyro.gx, raw_sensors.accelgyro.gy, raw_sensors.accelgyro.gz),
            Eigen::Vector3f(raw_sensors.accelgyro.ax, raw_sensors.accelgyro.ay, raw_sensors.accelgyro.az),
            Eigen::Vector3f(raw_sensors.accel.ax,     raw_sensors.accel.ay,     raw_sensors.accel.az),
            Eigen::Vector3f(raw_sensors.mag.mx,       raw_sensors.mag.my,       raw_sensors.mag.mz),
            raw_sensors.baro.press,
            raw_sensors.baro.temp,
            raw_sensors.gps
        );

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

void Estimator::updateState()
{
    // ── Orientation ───────────────────────────────────────────────────────────
    const Eigen::Vector4f q         = m_ekf.quaternion();
    m_state.orientation             = Eigen::Quaternionf(q(0), q(1), q(2), q(3));

    // ── Navigation states ─────────────────────────────────────────────────────
    m_state.position                = m_ekf.position();
    m_state.velocity                = m_ekf.velocity();
    
    m_state.gpsPosition            = m_ekf.gpsPosition();

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

    // ── Calibration ───────────────────────────────────────────────────────────
    m_state.accelBiases             = m_ekf.accelBias();
    m_state.gyroBiases              = m_ekf.gyroBias();
    m_state.calibration_quality     = m_calibrator.getCalibrationQuality();

    m_state.highGBiases             = m_calibrator.getHighGBiases();
    m_state.refMag                  = m_calibrator.getMagRef();
}