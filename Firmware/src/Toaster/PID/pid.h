#pragma once

#include <cstdint>
#include <functional>

/**
 * @brief PID controller.
 */
class PID {
public:
    PID(const double kp, const double ki = 0.0, const double kd = 0.0, const double integralLimit = 0.0, const double derivativeLimit = 0.0);

    void setErrorFunction(std::function<double(double, double)> errorFunction);
    void setDerivativeFunction(std::function<double(double, double, double)> derivativeFunction);

    /**
     * @brief Run an update step on the PID control loop.
     *
     * @param target Target value.
     * @param measurement Measurement value.
     * @param dt Timestep in seconds.
     * @return double Control value.
     */
    double update(const double target, const double measurement, const double dt);

    /// @brief Reset accumulators.
    void reset();

    struct Log {
        double measurement;
        double target;
        double error;
        double kp;
        double ki;
        double kd;
        double control;
        uint64_t time;
    };

    /// @brief Get the most recent logging variables.
    const Log& getLog();

private:
    static double defaultErrorFunction(const double target, const double measurement);
    static double defaultDerivativeFunction(const double measurement, const double prevMeasurement, const double dt);

    const double m_kp;
    const double m_ki;
    const double m_kd;

    const double m_integralLimit;
    const double m_derivativeLimit;

    std::function<double(double, double)> m_errorFunction { std::bind(defaultErrorFunction, std::placeholders::_1, std::placeholders::_2) };
    std::function<double(double, double, double)> m_derivativeFunction { std::bind(defaultDerivativeFunction, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3) };

    double m_prevMeasurement { 0.0 };
    double m_integral { 0.0 };

    uint32_t m_updateCount { 0 };

    Log m_log;
};