#include "pid.h"

#include <Arduino.h>

PID::PID(const double kp, const double ki, const double kd, const double integralLimit, const double derivativeLimit)
    : m_kp(kp), m_ki(ki), m_kd(kd), m_integralLimit(integralLimit), m_derivativeLimit(derivativeLimit) {}

void PID::setErrorFunction(const std::function<double(double, double)> errorFunction) {
    m_errorFunction = errorFunction;
}

void PID::setDerivativeFunction(const std::function<double(double, double, double)> derivativeFunction) {
    m_derivativeFunction = derivativeFunction;
}

double PID::update(const double target, const double measurement, const double dt) {
    const double error = m_errorFunction(target, measurement);

    // Proportional term.
    double pOut = m_kp * error;
    double iOut { 0 };
    double dOut { 0 };

    if (dt > 0) {
        // Integral term
        m_integral += error * dt;

        // Clamp integral to prevent windup
        if (m_integral > m_integralLimit) {
            m_integral = m_integralLimit;
        } else if (m_integral < -m_integralLimit) {
            m_integral = -m_integralLimit;
        }

        iOut = m_ki * m_integral;

        // Derivative Term
        if (m_updateCount > 0) {
            double derivative = m_derivativeFunction(measurement, m_prevMeasurement, dt);

            // Clamp derivative to prevent jerks in motion from causing massive
            // derivative control
            if (derivative > m_derivativeLimit) {
                derivative = m_derivativeLimit;
            } else if (derivative < -m_derivativeLimit) {
                derivative = -m_derivativeLimit;
            }

            dOut = -m_kd * derivative;
        }

        m_prevMeasurement = measurement;
    }

    const double control = pOut + iOut + dOut;

    // Update log
    m_log.target = target;
    m_log.measurement = measurement;
    m_log.error = error;
    m_log.kp = pOut;
    m_log.ki = iOut;
    m_log.kd = dOut;
    m_log.control = control;
    m_log.time = millis();

    m_updateCount++;

    // Return total output
    return control;
}

void PID::reset() {
    m_integral = 0.0;
    m_prevMeasurement = 0.0;
    m_updateCount = 0;
}

const PID::Log& PID::getLog() {
    return m_log;
}

double PID::defaultErrorFunction(const double target, const double measurement) {
    return target - measurement;
}

double PID::defaultDerivativeFunction(const double measurement, const double prevMeasurement, const double dt) {
    return (measurement - prevMeasurement) / dt;
}
