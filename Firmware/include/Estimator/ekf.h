#pragma once

#include <Eigen/Dense>
#include <ArduinoJson.h>
#include <string>

#include <libriccore/riccorelogging.h>
#include <librrc/Helpers/jsonconfighelper.h>

#include "Config/types.h"
#include "Config/systemflags_config.h"
#include "Sensors/sensors.h"
#include "Sensors/sensorStructs.h"

// ── State vector layout (22 states) ──────────────────────────────────────────
//
//  Index  Size  Description
//  -----  ----  -----------
//   0- 2    3   position NED (m)
//   3- 5    3   velocity NED (m/s)
//   6- 9    4   attitude quaternion [q0 q1 q2 q3] (scalar first NED -> Body)
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
    void update(const Eigen::Vector3f       gyro, 
                const Eigen::Vector3f       accel, 
                const Eigen::Vector3f       h_accel, 
                const Eigen::Vector3f       mag,
                const float                 pressure,
                const float                 temperature,
                const SensorStructs::GPS_t& gps
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

    Eigen::Vector3f magInnovation()     const { return m_y.segment<3>(0);  }
    Eigen::Vector3f accelInnovation()   const { return m_y.segment<3>(3);  }
    Eigen::Vector2f baroInnovation()    const { return m_y.segment<2>(6);  }
    Eigen::Vector3f gpsPosInnovation()  const { return m_y.segment<3>(8);  }
    Eigen::Vector3f gpsVelInnovation()  const { return m_y.segment<3>(11); }

    Eigen::Vector3f gpsPosition()   const { return m_gps_position; }


private:
    uint32_t m_lastPredictTime = 0;

    Eigen::Vector3f m_h_accel_bias{0,0,0};                  // (m/s^2) (body)
    Eigen::Vector3f m_mag_ref{1,0,0};                       // NED reference field (unit)
    SensorStructs::home_ref_t m_setHome_ref;

    Eigen::Vector3f m_acceleration;                         // body frame (m/s^2)
    Eigen::Vector3f m_angular_rates;                        // body frame (rad/s)
    
    static constexpr float g = 9.80665f;
    static constexpr float LOW_G_SATURATION  = 7.0f * g; 

    // ── State and covariance ──────────────────────────────────────────────────
    Eigen::Matrix<float, 16, 1>   m_x;   // state vector
    Eigen::Matrix<float, 16, 16>  m_P;   // covariance matrix
    Eigen::Matrix<float, 14, 1>   m_h;   // expected sensor reading vector  (mag, accel, baro, gps_pos, gps_vel)
    Eigen::Matrix<float, 14, 1>   m_y;   // innovation vector               (mag, accel, baro, gps_pos, gps_vel)
    

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
    static constexpr float SIGMA_JERK       = 1.0f;     // m/s³
    static inline const Eigen::Vector3f SIGMA_ALPHA{0.02f, 0.02f, 0.02f};  // rad/s 

    static constexpr float SIGMA_BG         = 1e-6f;     // rad/s 
    static constexpr float SIGMA_BA_LOW     = 1e-6f;     // m/s²
    static inline const Eigen::Vector3f SIGMA_ACCEL_LOW{0.1f, 0.4f, 0.45f};
    static constexpr float SIGMA_MAG        = 0.1f;     // was 0.01

    static constexpr float SIGMA_T          = 0.5f;      // K
    static constexpr float SIGMA_P          = 100.0f;    // Pa

    static constexpr float SIGMA_VEL        = 0.1f;      // m/s — tune to your GPS spec

    void predict(   const float dt, 
                    const Eigen::Vector3f gyro,
                    const Eigen::Vector3f accel,
                    const Eigen::Vector3f h_accel
                );
    void updateMag(const Eigen::Vector3f& z_meas_raw);
    void updateLowGAccel(const Eigen::Vector3f& z_accel);
    void updateBaro(const float pressure, const float temperature);
    void updateGPS(const SensorStructs::GPS_t& gps);

    // ── Atmospheric model constants ───────────────────────────────────────────
    static constexpr float BARO_M_0   = 0.0289644f;  // kg/mol  molar mass of air
    static constexpr float BARO_R_GAS = 8.31446f;    // J/(mol·K) universal gas constant
    static constexpr float BARO_L     = -0.0065f;    // K/m  ISA lapse rate

    // ── WGS84 constants ───────────────────────────────────────────────────────
    static constexpr double GPS_A_EARTH = 6378137.0;
    static constexpr double GPS_E2      = 0.00669437999014;

    Eigen::Vector3f m_gps_position;

};
