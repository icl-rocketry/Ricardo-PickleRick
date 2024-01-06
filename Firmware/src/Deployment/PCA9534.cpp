#include "PCA9534.h"

#include <stdint.h>

#include <Wire.h>

#include <libriccore/threading/riccorethread.h>
#include <libriccore/threading/scopedlock.h>



void PCA9534::pinMode(uint8_t pin, PINMODE mode)
{
    if (pin > 7)
    {
        return;
    }
    
    RicCoreThread::ScopedLock sl(device_lock);

    switch(mode){
        case PINMODE::GPIO_OUTPUT:
        {
            configShadow &= ~(1 << pin);
            break;
        }
        case PINMODE::GPIO_INPUT:
        {
            configShadow |= (1 << pin);
            polarityShadow &= ~(1 << pin);
            break;
        }
        case PINMODE::GPIO_INPUT_INVERT:
        {
            configShadow |= (1 << pin);
            polarityShadow |= (1 << pin);
            break;
        }
        default:
        {

            break;
        }

    }

    writeRegister(POLARITY,polarityShadow);
    writeRegister(CONFIG,configShadow);

};

void PCA9534::digitalWrite(uint8_t pin, uint8_t level)
{
    if (pin > 7)
    {
        return;
    }

    RicCoreThread::ScopedLock sl(device_lock);
   
    switch (level)
    {
        case 0:
        {
            outputShadow &= ~(1 << pin);
            break;
        }
        case 1:
        {
            outputShadow |= (1 << pin);
            break;
        }
        default:
        {
            break;
        }
    }


    writeRegister(OUTPUT_PORT,outputShadow);

};

int PCA9534::digitalRead(uint8_t pin)
{
    if (pin > 7)
    {
        return 0;
    }

    return readRegister(INPUT_PORT) & ( 1 << pin);
};

bool PCA9534::alive()
{
    m_wire.beginTransmission(m_address);
    return !(m_wire.endTransmission()); // returns 0 if no error 
}

uint8_t PCA9534::readRegister(uint8_t reg)
{

    m_wire.beginTransmission(m_address);
    m_wire.write(reg);
    m_wire.endTransmission(false);

    m_wire.requestFrom(m_address,static_cast<uint8_t>(1));
    if (m_wire.available()){
        return m_wire.read();
    }else{
        return 0;
    }

};

size_t PCA9534::writeRegister(uint8_t reg, uint8_t data)
{

    m_wire.beginTransmission(m_address);
    m_wire.write(reg);
    size_t num_bytes = m_wire.write(data);
    m_wire.endTransmission();
    return num_bytes;

};

 