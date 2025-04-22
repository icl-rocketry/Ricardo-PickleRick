#pragma once

#include <ArduinoJson.h>

#include "MadgwickAHRS.h"

#include "Config/types.h"

#include "localizationkf.h"

#include "sensors.h"
#include "sensorStructs.h"



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
        void configure(Eigen::Quaternionf Orientation);
        void update(const SensorStructs::raw_measurements_t& raw_sensors);

        void setHome(const SensorStructs::raw_measurements_t& raw_sensors); //records the current position as the launch site
        bool isHomeSet(){return _homeSet;};
        
        void changeBeta(float beta);
        void refreshOrientation();
        void resetLocalization();
        void resetOrientation();

        void setFlightTime(uint32_t time);

        const SensorStructs::state_t& getData();

        
    private:
        // stateMachine* _sm;//pointer to statemachine object
        Types::CoreTypes::SystemStatus_t& _systemstatus;

        SensorStructs::state_t state;

        //time variables
        unsigned long last_update;
        unsigned long update_frequency;

        bool _homeSet;
        
        
        //ORIENTATION ESTIMATION
        Madgwick madgwick; // madgwick filter object
        static constexpr float g = 9.81;
        /**
         * @brief Reference orientation of board in the rocket
         * 
         */
        Eigen::Quaternionf refOrientation;
        /**
         * @brief Default orientation of board in rocket, this corresponds to roll along x axis, pitch along y axis and yaw along z axis in a RH coordinate frame.
         * 
         */
        // const Eigen::Quaternionf defaultOrientation(1.0,0.0,0.0,0.0);
        
        //POSITION ESTIMATION
        LocalizationKF localizationkf;

        //barometer methods
        void baroUpdate(const float& altitude);
        static constexpr float BARO_MAX_ALT = 10000; // 10km max alt 

        //private methods

        void computeOrientation(const float &gx, const float &gy, const float &gz,
                               const float &ax, const float &ay, const float &az,
                               const float &mx, const float &my, const float &mz, float dt);

        void computeOrientation(const float &gx, const float &gy, const float &gz,
                               const float &ax, const float &ay, const float &az, float dt);

        void updateOrientation();

        void updateAngularRates(const float &gx, const float &gy, const float &gz);

        Eigen::Vector3f getLinearAcceleration(const float& ax,const float& ay, const float& az);

        void changeEstimatorState(ESTIMATOR_STATE state,std::string logmessage);

        /**
         * @brief generate prediction from localization kf and assign result to state
         * 
         * @param dt delta t in seconds
         */
        void predictLocalizationKF(const float& dt);
        
        /**
         * @brief Calculates angle of nutation i.e tilt angle. We assume rocket axes convention here,
         *        i.e the roll axis is aligned with the vertical axis of the rocket, hence we use pitch and yaw
         *        to calcualte the tilt angle.
         * 
         * @param euler expects euler angle vector in the roll pitch yaw format in rad
         * @return float angle of nutation (tilt) [rad]
         */
        float calculateNutation(const Eigen::Vector3f &euler);

        void setRefOrientation(JsonObjectConst conf);

        Eigen::Vector3f quat2rpy(Eigen::Quaternionf quat);
};