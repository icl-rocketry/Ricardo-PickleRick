#pragma once

class Butterworth2Lowpass
{
public:
    Butterworth2Lowpass();

    void setup(float sample_rate_hz, float cutoff_hz);
    void reset(float value = 0.0f);
    float update(float x);

    bool valid() const;

private:
    void computeCoefficients();

    float m_fs;
    float m_fc;
    bool  m_valid;
    bool  m_initialized;

    float m_b0, m_b1, m_b2;
    float m_a1, m_a2;

    float m_x1, m_x2;
    float m_y1, m_y2;
};