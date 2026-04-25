#pragma once
#include <cmath>

class FirstOrderLowpass {
public:
    FirstOrderLowpass() = default;

    // Call once in setup
    void setup(float sample_rate_hz, float cutoff_hz) {
        float dt = 1.0f / sample_rate_hz;
        float rc = 1.0f / (2.0f * M_PI * cutoff_hz);

        m_alpha = dt / (rc + dt);
        m_initialized = false;
    }

    // Same style as your Butterworth filter
    float update(float x) {
        if (!m_initialized) {
            m_y = x;
            m_initialized = true;
            return m_y;
        }

        m_y = m_y + m_alpha * (x - m_y);
        return m_y;
    }

    // Optional: reset filter state
    void reset(float value = 0.0f) {
        m_y = value;
        m_initialized = false;
    }

private:
    float m_alpha = 1.0f;
    float m_y = 0.0f;
    bool  m_initialized = false;
};