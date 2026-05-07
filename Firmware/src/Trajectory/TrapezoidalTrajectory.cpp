#include "Trajectory/TrapezoidalTrajectory.h"

#include <algorithm>
#include <cmath>

namespace Trajectory {

bool TrapezoidalTrajectory::configure(const Eigen::Vector3f& start,
                                      const Eigen::Vector3f& finish,
                                      float duration_s,
                                      float accel_fraction)
{
    m_start = start;
    m_finish = finish;
    m_duration_s = duration_s;

    const Eigen::Vector3f delta = m_finish - m_start;
    m_distance_m = delta.norm();

    m_direction.setZero();
    m_accel_time_s = 0.0f;
    m_cruise_time_s = 0.0f;
    m_decel_start_s = 0.0f;
    m_peak_speed_mps = 0.0f;
    m_accel_mps2 = 0.0f;
    m_configured = false;

    if (m_duration_s <= 0.0f) {
        return false;
    }

    if (m_distance_m <= 1e-6f) {
        m_configured = true;
        return true;
    }

    accel_fraction = std::clamp(accel_fraction, 1e-3f, 0.499f);

    m_direction = delta / m_distance_m;
    m_accel_time_s = m_duration_s * accel_fraction;
    m_cruise_time_s = m_duration_s - (2.0f * m_accel_time_s);
    m_decel_start_s = m_accel_time_s + m_cruise_time_s;

    m_peak_speed_mps = m_distance_m / (m_duration_s - m_accel_time_s);
    m_accel_mps2 = m_peak_speed_mps / m_accel_time_s;
    m_configured = true;
    return true;
}

TrajectoryPoint TrapezoidalTrajectory::sample(float elapsed_s) const
{
    TrajectoryPoint point;

    if (!m_configured) {
        return point;
    }

    const float t = std::clamp(elapsed_s, 0.0f, m_duration_s);

    float distance = 0.0f;
    float speed = 0.0f;
    float accel = 0.0f;

    if (m_distance_m <= 1e-6f) {
        distance = 0.0f;
    } else if (t < m_accel_time_s) {
        accel = m_accel_mps2;
        speed = accel * t;
        distance = 0.5f * accel * t * t;
    } else if (t < m_decel_start_s) {
        accel = 0.0f;
        speed = m_peak_speed_mps;
        distance = (0.5f * m_accel_mps2 * m_accel_time_s * m_accel_time_s)
                 + (m_peak_speed_mps * (t - m_accel_time_s));
    } else {
        const float t_decel = t - m_decel_start_s;
        accel = -m_accel_mps2;
        speed = std::max(0.0f, m_peak_speed_mps - (m_accel_mps2 * t_decel));
        distance = (0.5f * m_accel_mps2 * m_accel_time_s * m_accel_time_s)
                 + (m_peak_speed_mps * m_cruise_time_s)
                 + (m_peak_speed_mps * t_decel)
                 - (0.5f * m_accel_mps2 * t_decel * t_decel);
    }

    if (t >= m_duration_s) {
        distance = m_distance_m;
        speed = 0.0f;
        accel = 0.0f;
    }

    point.position = m_start + (m_direction * std::clamp(distance, 0.0f, m_distance_m));
    point.velocity = m_direction * speed;
    point.acceleration = m_direction * accel;
    point.finished = elapsed_s >= m_duration_s;
    return point;
}

} // namespace Trajectory
