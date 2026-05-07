#pragma once

#include <Eigen/Dense>

namespace Trajectory {

struct TrajectoryPoint {
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
    Eigen::Vector3f acceleration = Eigen::Vector3f::Zero();
    bool finished = true;
};

class TrapezoidalTrajectory {
public:
    bool configure(const Eigen::Vector3f& start,
                   const Eigen::Vector3f& finish,
                   float duration_s,
                   float accel_fraction = 0.25f);

    TrajectoryPoint sample(float elapsed_s) const;

    float duration() const { return m_duration_s; }
    float accelTime() const { return m_accel_time_s; }
    float cruiseTime() const { return m_cruise_time_s; }
    float peakSpeed() const { return m_peak_speed_mps; }
    bool isConfigured() const { return m_configured; }

private:
    Eigen::Vector3f m_start = Eigen::Vector3f::Zero();
    Eigen::Vector3f m_finish = Eigen::Vector3f::Zero();
    Eigen::Vector3f m_direction = Eigen::Vector3f::Zero();

    float m_distance_m = 0.0f;
    float m_duration_s = 0.0f;
    float m_accel_time_s = 0.0f;
    float m_cruise_time_s = 0.0f;
    float m_decel_start_s = 0.0f;
    float m_peak_speed_mps = 0.0f;
    float m_accel_mps2 = 0.0f;
    bool m_configured = false;
};

} // namespace Trajectory
