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
    m_calibrator.setup();
    m_ekf.setup(
        m_calibrator.getGyroBiases(),
        m_calibrator.getAccelBiases(),
        m_calibrator.getMagRef()
    );
};

void Estimator::update(const SensorStructs::raw_measurements_t &raw_sensors)
{


    if (m_calibrating) {
        if (m_calibrator.getNumberOfMeasurements() >= 10 * 500) {
            m_calibrator.compute();
            m_ekf.setup(
                m_calibrator.getGyroBiases(),
                m_calibrator.getAccelBiases(),
                m_calibrator.getMagRef()
            );
            m_calibrating = false;
        } else {
            m_calibrator.update(raw_sensors);
        }
    } else {
        const Eigen::Vector3f gyro(
            raw_sensors.accelgyro.gx,
            raw_sensors.accelgyro.gy,
            raw_sensors.accelgyro.gz
        );
        const Eigen::Vector3f accel(
            raw_sensors.accelgyro.ax,
            raw_sensors.accelgyro.ay,
            raw_sensors.accelgyro.az
        );
        const Eigen::Vector3f mag(
            raw_sensors.mag.mx,
            raw_sensors.mag.my,
            raw_sensors.mag.mz
        );
        updateOrientation(gyro, accel, mag);

    }

    updateState();


};

void Estimator::calibrate()
{
    m_calibrating = true;
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Calibration started");

};
void Estimator::setHome(const SensorStructs::raw_measurements_t &raw_sensors)
{
    //
    m_homeSet = true;
};

void Estimator::updateOrientation(  const Eigen::Vector3f gyro, 
                                    const Eigen::Vector3f accel, 
                                    const Eigen::Vector3f mag)
{
    m_ekf.update(gyro, accel, mag);
};

void Estimator::updateState()
{
    // ── Orientation ───────────────────────────────────────────────────────────
    const Eigen::Vector4f q         = m_ekf.quaternion();
    m_state.orientation             = Eigen::Quaternionf(q(0), q(1), q(2), q(3));

    // ── Navigation states ─────────────────────────────────────────────────────
    m_state.position                = m_ekf.position();
    m_state.velocity                = m_ekf.velocity();
    m_state.acceleration            = m_ekf.acceleration();
    
    // ── Expected Readings ─────────────────────────────────────────────────────
    m_state.expectedMagReading      = m_ekf.expectedMagReading();
    m_state.expectedAccelReading    = m_ekf.expectedAccelReading();

    // ── Innovation  ───────────────────────────────────────────────────────────
    m_state.magInnovation           = m_ekf.magInnovation();
    m_state.accelInnovation         = m_ekf.accelInnovation();

    // ── Calibration ───────────────────────────────────────────────────────────
    m_state.accelBiases             = m_ekf.accelBias();
    m_state.gyroBiases              = m_ekf.gyroBias();
    m_state.calibration_quality     = m_calibrator.getCalibrationQuality();

    m_state.highGBiases             = m_calibrator.getHighGBiases();
    m_state.refMag                  = m_calibrator.getMagRef();
}