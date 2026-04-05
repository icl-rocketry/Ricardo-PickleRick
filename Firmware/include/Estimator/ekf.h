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
//   6- 8    3   acceleration NED (m/s²)
//   9-12    4   attitude quaternion [q0 q1 q2 q3] (scalar first NED -> Body)
//  13-15    3   low-g accel bias (m/s²)
//
// Note: high-g accel bias excluded from update but handled in calibrate.
// ─────────────────────────────────────────────────────────────────────────────

class EKF
{
public:
    EKF() {};

    void setup(const Eigen::Vector3f& gyro_bias, const Eigen::Vector3f& accel_bias, const Eigen::Vector3f& mag_ref);
    void update(const Eigen::Vector3f gyro, const Eigen::Vector3f accel, const Eigen::Vector3f mag);

    // ── State accessors ───────────────────────────────────────────────────────
    Eigen::Vector3f position()      const { return m_x.segment<3>(0);  }
    Eigen::Vector3f velocity()      const { return m_x.segment<3>(3);  }
    Eigen::Vector3f acceleration()  const { return m_x.segment<3>(6);  }
    Eigen::Vector4f quaternion()    const { return m_x.segment<4>(9);  }
    Eigen::Vector3f accelBias()     const { return m_x.segment<3>(13); }

    Eigen::Vector3f gyroBias()      const { return m_gyro_bias; }
    
    Eigen::Vector3f expectedMagReading()     const { return m_h.segment<3>(0);  }
    Eigen::Vector3f expectedAccelReading()   const { return m_h.segment<3>(3);  }

    Eigen::Vector3f magInnovation()     const { return m_y.segment<3>(0);  }
    Eigen::Vector3f accelInnovation()   const { return m_y.segment<3>(3);  }
private:
    uint32_t m_lastPredictTime = 0;

    Eigen::Vector3f m_mag_ref{1,0,0};  // NED reference field (unit)
    Eigen::Vector3f m_gyro_bias{0,0,0};  // body frame (rad/s)
    static constexpr float g = 9.80665f;

    // ── State and covariance ──────────────────────────────────────────────────
    Eigen::Matrix<float, 16, 1>   m_x;   // state vector
    Eigen::Matrix<float, 16, 16>  m_P;   // covariance matrix
    Eigen::Matrix<float, 6, 1>    m_h;   // expected sensor reading vector  (mag, accel)
    Eigen::Matrix<float, 6, 1>    m_y;   // innovation vector               (mag, accel)
    

    Eigen::Matrix<float, 16, 16>  m_F;      // state transition
    Eigen::Matrix<float, 16, 16>  m_Q;      // process noise
    Eigen::Matrix<float, 16, 3>   m_K;      // Kalman gain
    Eigen::Matrix<float, 16, 16>  m_IKH;    // (I - KH) for Joseph form

    // ── Predict intermediates ─────────────────────────────────────────────────
    Eigen::Matrix<float, 9, 9>    m_F_trans;
    Eigen::Matrix<float, 9, 9>    m_Q_trans;
    Eigen::Matrix<float, 4, 4>    m_F_att;
    Eigen::Matrix<float, 4, 4>    m_Q_att;

    // ── Shared measurement Jacobian (reused across all update steps) ──────────
    Eigen::Matrix<float, 3, 16>   m_H;

    // ── Scratch space for matrix products — avoids 22×22 stack temporaries ───
    Eigen::Matrix<float, 16, 16>  m_P_temp;

    // ── Process noise tuning ──────────────────────────────────────────────────
    // Tune these to match your IMU characteristics
    static constexpr float SIGMA_JERK       = 0.5f;     // m/s³
    static constexpr float SIGMA_ALPHA      = 0.005f;   // rad/s 
    static constexpr float SIGMA_BG         = 0.0f;     // rad/s/√s (bias frozen for now)
    static constexpr float SIGMA_BA_LOW     = 0.0f;     // m/s²/√s  (bias frozen for now)
    
    static constexpr float SIGMA_MAG        = 0.1f;     // 
    static constexpr float SIGMA_ACCEL_LOW  = 0.05f;    // 

    void predict(const float dt, const Eigen::Vector3f gyro);
    void updateMag(const Eigen::Vector3f& z_meas_raw);
    void updateLowGAccel(const Eigen::Vector3f& z_accel);

};
