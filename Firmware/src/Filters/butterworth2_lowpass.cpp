#include "Filters/butterworth2_lowpass.h"
#include <cmath>

Butterworth2Lowpass::Butterworth2Lowpass()
    : m_fs(0.0f),
      m_fc(0.0f),
      m_valid(false),
      m_initialized(false),
      m_b0(0.0f), m_b1(0.0f), m_b2(0.0f),
      m_a1(0.0f), m_a2(0.0f),
      m_x1(0.0f), m_x2(0.0f),
      m_y1(0.0f), m_y2(0.0f)
{
}

void Butterworth2Lowpass::setup(float sample_rate_hz, float cutoff_hz)
{
    m_fs = sample_rate_hz;
    m_fc = cutoff_hz;

    if (m_fs <= 0.0f || m_fc <= 0.0f || m_fc >= 0.5f * m_fs)
    {
        m_valid = false;
        return;
    }

    computeCoefficients();
    reset(0.0f);
    m_valid = true;
}

void Butterworth2Lowpass::reset(float value)
{
    m_x1 = value;
    m_x2 = value;
    m_y1 = value;
    m_y2 = value;
    m_initialized = false;
}

float Butterworth2Lowpass::update(float x)
{
    if (!m_valid)
        return x;

    if (!m_initialized)
    {
        m_x1 = x;
        m_x2 = x;
        m_y1 = x;
        m_y2 = x;
        m_initialized = true;
    }

    const float y =
          m_b0 * x
        + m_b1 * m_x1
        + m_b2 * m_x2
        - m_a1 * m_y1
        - m_a2 * m_y2;

    m_x2 = m_x1;
    m_x1 = x;
    m_y2 = m_y1;
    m_y1 = y;

    return y;
}

bool Butterworth2Lowpass::valid() const
{
    return m_valid;
}

void Butterworth2Lowpass::computeCoefficients()
{
    const float pi = 3.14159265358979323846f;
    const float omega = 2.0f * pi * m_fc / m_fs;
    const float cos_omega = std::cos(omega);
    const float sin_omega = std::sin(omega);

    const float Q = 1.0f / std::sqrt(2.0f);
    const float alpha = sin_omega / (2.0f * Q);

    const float b0 = 0.5f * (1.0f - cos_omega);
    const float b1 = 1.0f - cos_omega;
    const float b2 = 0.5f * (1.0f - cos_omega);
    const float a0 = 1.0f + alpha;
    const float a1 = -2.0f * cos_omega;
    const float a2 = 1.0f - alpha;

    m_b0 = b0 / a0;
    m_b1 = b1 / a0;
    m_b2 = b2 / a0;
    m_a1 = a1 / a0;
    m_a2 = a2 / a0;
}