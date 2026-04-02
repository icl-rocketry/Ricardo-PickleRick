#pragma once
/**
 * @brief Class implementing the GPIO HAL interface requried for NRCRemotePyro. Look at librrc/HAL/ArduinoGpio for reference.
 * 
 */

#include "PCA9534.h"



class PCA9534Gpio
{
    public:
        using PINMODE = PCA9534::PINMODE;

        PCA9534Gpio(PCA9534& pca,uint8_t pin):
        m_pca(pca),
        m_pin(pin)
        {};

        void pinMode(PINMODE mode)
        {
            m_pca.pinMode(m_pin,mode);
        };

        void digitalWrite(uint8_t level)
        {
            m_pca.digitalWrite(m_pin,level);
        };

        int digitalRead()
        {
            return m_pca.digitalRead(m_pin);
        };

    private:
        PCA9534& m_pca;
        const uint8_t m_pin;


};