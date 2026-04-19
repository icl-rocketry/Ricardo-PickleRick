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