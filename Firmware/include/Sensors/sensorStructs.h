#pragma once
/*
definition of structs used within sensor classes
*/

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace SensorStructs
{

    struct ACCELGYRO_6AXIS_t{
        float ax; // (m/s^2)
        float ay; // (m/s^2)
        float az; // (m/s^2)
        float gx; // (rad/s)
        float gy; // (rad/s)
        float gz; // (rad/s)

        float temp;
    };

    struct ACCEL_3AXIS_t{
        float ax; // (m/s^2)
        float ay; // (m/s^2)
        float az; // (m/s^2)
    };

    struct MAG_3AXIS_t{
        float mx; // (normalised to 1 when calibrated)
        float my; // (normalised to 1 when calibrated)
        float mz; // (normalised to 1 when calibrated)

        float temp;
    };
    
    struct BARO_t{
        float temp;     // Kelvin
        float press;    // Pa
    };
    
    struct GPS_t{

        // Keep as raw integer — float loses precision at real-world coordinates
        int32_t latitude;       // degrees * 1e-7  (divide by 1e7f only when needed)
        int32_t longitude;      // degrees * 1e-7
        float   altitude;       // m from mean sea level

        float   v_n;            // m/s
        float   v_e;            // m/s
        float   v_d;            // m/s

        float   hAcc;           // m horizontal accuracy
        float   vAcc;           // m vertial accuracy
        
        uint8_t sat;            // number of satilites
        uint8_t fix;            // gps fix type
        bool    updated;        // flag if gps values have been updated
        bool    valid;
    };

    struct ADC_V_RAIL_t{

        int volt;       // mV
        int percent;    // Percentage in reference to max voltage expeceted 
    };

    struct INA_V_RAIL_t{

        int volt;       // mV
        int current;    // mA
        int power;      // mW
        int percent;    // Percentage in reference to max voltage expeceted 

    };

    struct raw_measurements_t
    {
        ACCELGYRO_6AXIS_t accelgyro;
        ACCEL_3AXIS_t accel;
        MAG_3AXIS_t mag;
        BARO_t baro;
        GPS_t gps;
        ADC_V_RAIL_t logicrail;
        INA_V_RAIL_t deprail;

        uint64_t system_time;
    };

    struct home_ref_t
    {
        // GPS
        int32_t launch_lat;
        int32_t launch_lon;
        float   launch_alt;
        // Baro
        float   launch_pressure;
        float   launch_temperature;
    };

    struct state_t
    {
        Eigen::Quaternionf orientation;         // (quaternion)                         (NED to Body)
        Eigen::Vector3f eulerAngles;            // (rad) (roll pitch yaw)

        Eigen::Vector3f position;               // (m) relative to callibration site    (NED)
        Eigen::Vector3f velocity;               // (m/s)                                (NED)
        
        Eigen::Vector3f gyroBiases;             // (rad/s)                              (body)
        Eigen::Vector3f accelBiases;            // (m/s^2)                              (body)
        
        uint8_t calibration_quality;
        
        Eigen::Vector3f gpsPosition;            // (m)                                  (NED)

        Eigen::Vector3f expectedMagReading;     // (unit direction vector)              (body)
        Eigen::Vector3f expectedAccelReading;   // (m/s^2)                              (body)
        Eigen::Vector2f expectedBaroReading;    // (Kelvin, Pascal)                     (NED)
        Eigen::Vector3f expectedGpsPosReading;  // (m)                                  (NED)
        Eigen::Vector3f expectedGpsVelReading;  // (m/s)                                (NED)
        
        Eigen::Vector3f magInnovation;          // (unit direction vector)              
        Eigen::Vector3f accelInnovation;        // (m/s^2)              
        Eigen::Vector2f baroInnovation;         // (Kelvin, Pascal)
        Eigen::Vector3f gpsPosInnovation;       // (m)
        Eigen::Vector3f gpsVelInnovation;       // (m/s)


        Eigen::Vector3f highGBiases;            // (m/s^2)                              (body)
        Eigen::Vector3f refMag;                 // (unit direction vector)              (NED)


        // Launch Site
        home_ref_t launch_ref;

        //times -> all must be initialized to zero
        uint32_t ignitionTime{0};
        uint32_t liftoffTime{0};
        uint32_t apogeeTime{0};

        /**
         * @brief Estimator state -> maybe change this to be a bitfield lol
         * 
         * 
            NOMINAL = 0
            PARTIAL_NO_IMU = 1
            PARTIAL_NO_IMU_NO_GPS = 2
            PARTIAL_NO_IMU_NO_BARO = 3
            PARTIAL_NO_MAG = 4
            PARTIAL_NO_GPS = 5
            PARTIAL_NO_GPS_NO_BARO = 6
            PARTIAL_NO_BARO = 7
            NO_HOME = 8
            NOSOLUTION = 9
         * 
         */
        uint8_t estimator_state;
    };


}
