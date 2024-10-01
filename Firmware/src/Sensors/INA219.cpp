#include "INA219.h"

void INA219::update()
{
    if (!m_initialized)
    {
        return;
    }

    if (millis() - _prevUpdate > _pollDelta)
    {
        int16_t currentReg = readRegister(INA219::Registers::Current);

        if (currentReg & signmask16bit)
        {
            currentReg |= signmask16bit;
        }

        //!TODO there is a constant current offset which depends on the bus voltage - compensation needs to be added for this.
        _current = static_cast<float>(_currentLSB * currentReg);

        uint16_t busVReg = readRegister(INA219::Registers::BusV);
        _busV = static_cast<float>(busVLSB * (busVReg >> 3));
        _prevUpdate = millis();
    }

    

}

bool INA219::setup(float resistance, float maxCurrent, INA219::PGAGain Gain, INA219::ADCSettings ShuntVADCsettings, INA219::ADCSettings BusVADCsettings, INA219::Modes DeviceMode, INA219::busVRange vRange)
{
    if (!alive())
    {
        return false;
    }

    reset();
    setBusVRange(vRange);
    setGain(Gain);
    setShuntVADC(ShuntVADCsettings);
    setBusVADC(BusVADCsettings);
    setMode(DeviceMode);
    setCalibration(resistance, maxCurrent); 

    m_initialized = true;   
    return true;
}

bool INA219::alive(){
    _wire.beginTransmission(_deviceAddr);
    return !_wire.endTransmission();
}

void INA219::writeRegister(INA219::Registers address, uint16_t value)
{
    _wire.beginTransmission(_deviceAddr);
    _wire.write(static_cast<uint8_t>(address));
    _wire.write(value >> 8);
    _wire.write(value & 0xFF);
    _wire.endTransmission();
}

int16_t INA219::readRegister(INA219::Registers address)
{

    _wire.beginTransmission(_deviceAddr);
    _wire.write(static_cast<uint8_t>(address));
    _wire.endTransmission();

    _wire.requestFrom(_deviceAddr, 2);
    int16_t readResult = _wire.read() << 8;
    readResult |= _wire.read();
    return readResult;
}

void INA219::setCalibration(float resistance, float maxCurrent)
{
    _currentLSB = maxCurrent * recip2pow15;

    _calibrationReg = static_cast<uint16_t>(std::trunc(0.04096 / (_currentLSB * resistance)));
    
    writeRegister(Registers::Calibration, _calibrationReg);
}

void INA219::setBusVRange(INA219::busVRange range)
{

    _configReg &= busVRangeMask;
    _configReg |= static_cast<bool>(range) << 13;

    writeRegister(Registers::Config, _configReg);
}

void INA219::setGain(INA219::PGAGain Gain)
{

    _configReg &= gainMask;
    _configReg |= static_cast<uint16_t>(Gain) << 11;

    writeRegister(Registers::Config, _configReg);
}

void INA219::setShuntVADC(INA219::ADCSettings ADCsetting)
{
    _configReg &= shuntADCMask;
    _configReg |= static_cast<uint16_t>(ADCsetting) << 3;

    writeRegister(Registers::Config, _configReg);
}

void INA219::setBusVADC(INA219::ADCSettings ADCsetting)
{
    _configReg &= busADCMask;
    _configReg |= static_cast<uint16_t>(ADCsetting) << 7;

    writeRegister(Registers::Config, _configReg);
}

void INA219::setMode(INA219::Modes Mode)
{
    _configReg &= modeMask;
    _configReg |= static_cast<uint16_t>(Mode);

    writeRegister(Registers::Config, _configReg);
}

void INA219::reset(){
    _configReg &= resetMask;
    _configReg |= 1 << 15;
    writeRegister(Registers::Config, _configReg);
}