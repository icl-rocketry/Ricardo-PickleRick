/**
 * @file PCA9685PWM.h
 * @author Kiran de Silva (kd619@ic.ac.uk)
 * @brief PCA9685 pwm hal
 * @version 0.1
 * @date 2024-09-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include <cstdint>
#include "PCA9685.h"
#include <libriccore/riccorelogging.h>


class PCA9685PWM
{
public:
    PCA9685PWM(uint8_t pin, PCA9685& pca) : m_pin(pin),
                                            m_pca(pca)     
    {};

    /**
     * @brief Change duty of pwm channel specifing duty within range [0,2^m_res]. MaxDuty = 2^m_res
     *      Hint: use LIBRRC::RangeMap to remap your required range to the range [0, MaxDuty]
     *
     * @param duty
     */
    void writeDuty(uint32_t duty)
    {
        m_pca.writeDuty(m_pin,duty);
    };

    /**
     * @brief Change duty cycle of pwm channel, specifiyng pulse width. 
     *        Hint: Keep track of what the underlying base frequency is...
     *
     * @param usec
     */
    void writeWidth(uint32_t usec)
    {
        m_pca.writeWidth(m_pin,usec);
    };

    /**
     * @brief Get the Returns the max duty of the pwm channel
     *
     * @return uint32_t
     */
    uint32_t getMaxDuty()
    {
        return std::pow(2, getResolution());
    };

    uint8_t getResolution()
    {
        return m_pca.getFrequency();
    }

    uint32_t getFrequency()
    {
        return m_pca.getFrequency();
    };

    void setPWMParam(uint32_t freq, uint8_t res){
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Warning! PCA9685 PWM does not support per channel resolution or freuqnecy settings!");
    };

private:
    const uint8_t m_pin;
    PCA9685& m_pca;

};