#include "PCA9685.h"



bool PCA9685::alive()
{
    m_wire.beginTransmission(m_address);
    return !(m_wire.endTransmission()); // returns 0 if no error 
}

bool PCA9685::setup()
{

    if (!alive())
    {
        return false;
    }

    //write default config
    writeRegister(OUTPUT_PORT,outputShadow);
    writeRegister(POLARITY,polarityShadow);
    writeRegister(CONFIG,configShadow);
    return true;


}

uint8_t PCA9685::readRegister(uint8_t reg)
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

size_t PCA9685::writeRegister(uint8_t reg, uint8_t data)
{

    m_wire.beginTransmission(m_address);
    m_wire.write(reg);
    size_t num_bytes = m_wire.write(data);
    m_wire.endTransmission();
    return num_bytes;

};

 