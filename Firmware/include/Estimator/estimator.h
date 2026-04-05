#pragma once

#include <ArduinoJson.h>
#include <string>

#include <libriccore/riccorelogging.h>
#include <librrc/Helpers/jsonconfighelper.h>

#include "Config/types.h"
#include "Config/systemflags_config.h"
#include "Sensors/sensors.h"
#include "Sensors/sensorStructs.h"

#include "Estimator/ekf.h"
#include "Estimator/calibrator.h"

enum class ESTIMATOR_STATE: uint8_t{
    NOMINAL,
    PARTIAL_NO_IMU,
    PARTIAL_NO_IMU_NO_GPS,
    PARTIAL_NO_IMU_NO_BARO,
    PARTIAL_NO_MAG,
    PARTIAL_NO_GPS,
    PARTIAL_NO_GPS_NO_BARO,
    PARTIAL_NO_BARO,
    NO_HOME,
    NOSOLUTION
};

class Estimator{
    public:
        Estimator(Types::CoreTypes::SystemStatus_t& systemstatus);   
        
        void setup();
        void update(const SensorStructs::raw_measurements_t& raw_sensors);

        void calibrate();
        void setHome(const SensorStructs::raw_measurements_t& raw_sensors); //records the current position as the launch site
        
        bool isHomeSet() { return m_homeSet; };
        const SensorStructs::state_t& getData() { return m_state; };

        
    private:
        Types::CoreTypes::SystemStatus_t& m_systemstatus;
        SensorStructs::state_t m_state;

        unsigned long m_last_update;
        unsigned long m_update_frequency;

        bool m_homeSet;        
        bool m_initialised; // has this device ever been calibrated (req for mag)
        bool m_calibrating;
        Eigen::Quaternionf m_refOrientation;
        
        EKF m_ekf;    
        Calibrator m_calibrator;
        void updateOrientation(const Eigen::Vector3f gyro, const Eigen::Vector3f accel, const Eigen::Vector3f mag);

        void updateState();
};
