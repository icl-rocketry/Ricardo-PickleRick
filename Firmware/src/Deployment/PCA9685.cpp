#include "PCA9685.h"
#include <math.h>
#include <libriccore/riccorelogging.h>


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

    //turn all outputs off
    writeRegister(MODE1, mode1Shadow);
    writeRegister(MODE2,mode2Shadow);
    writeTiming(ALL_LED_START,0,500);
   
    setFrequency(m_freq);

    uint8_t reg = readRegister(MODE1);

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("HI");
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("pwm reg " + std::to_string(reg));
    
    return true;


}

void PCA9685::writeDuty(uint8_t pin, uint32_t duty){
    if (pin >= MAX_CHANNEL)
    {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Illegal Pin requested!");
        return;
    }
    duty = std::min(std::max(duty,static_cast<uint32_t>(0)),static_cast<uint32_t>(4095));
    uint8_t address = LEDSTART + (4*pin);
    if (duty == 0)
    {
        writeTiming(address,0,4096);
    }
    else if (duty == 4095)
    {
        writeTiming(address,4096,0);
    }
    else
    {
        writeTiming(address,0,duty);
    }

}

void PCA9685::writeWidth(uint8_t pin, uint32_t usec)
{
    writeDuty(pin,toCounts(usec));
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

uint32_t PCA9685::toCounts(uint32_t usec)
{
    uint32_t maxDuty = 4096;
    float fractionOfPeriod = (static_cast<float>(usec) * static_cast<float>(m_freq)) / static_cast<float>(1e6);
    float duty = fractionOfPeriod * static_cast<float>(maxDuty);

    return static_cast<uint32_t>(duty);

}

void PCA9685::writeTiming(uint8_t startAddress,uint16_t on, uint16_t off)
{
    m_wire.beginTransmission(m_address);
    m_wire.write(startAddress);
    m_wire.write(static_cast<uint8_t>(on));
    m_wire.write(static_cast<uint8_t>(on >> 8));
    m_wire.write(static_cast<uint8_t>(off));
    m_wire.write(static_cast<uint8_t>(off >> 8));
    m_wire.endTransmission();

}

void PCA9685::setFrequency(uint32_t freq)
{
    // min is 0x03
    //max is 0xff

    float prescale_temp = std::ceil(( static_cast<float>(OSC_FREQUENCY) / (static_cast<float>(4096) * static_cast<float>(freq)) ) - 1);
    prescale_temp = std::min(std::max(prescale_temp,0.f),255.f); // ensure float is properly bounded for uint8_t
    // bound prescale to hardware limits
    uint8_t prescale = std::min(std::max(static_cast<uint8_t>(prescale_temp),static_cast<uint8_t>(0xff)),static_cast<uint8_t>(0x03));
    
    uint8_t newmode = (mode1Shadow & ~RESTART) | SLEEP;
    writeRegister(MODE1,newmode);
    writeRegister(PRE_SCALE,prescale);
    writeRegister(MODE1,mode1Shadow);
    delay(5);
    //ensure auto increment is on
    writeRegister(MODE1, mode1Shadow | RESTART | AUTO_INC);

}