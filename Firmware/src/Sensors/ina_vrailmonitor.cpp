#include "ina_vrailmonitor.h"

#include <string>

#include <Wire.h>

#include <libriccore/riccorelogging.h>

#include "sensorStructs.h"

#include "INA219.h"


INA_VRailMonitor::INA_VRailMonitor(std::string_view vrail_name,TwoWire& wire, uint8_t address):
    _name(vrail_name),
    m_ina(address,wire),
    _maxVoltage(0),
    _lowVoltage(0),
    _minVoltage(0)
{};

void INA_VRailMonitor::setup(int maxVoltage, int lowVoltage,int minVoltage){
    _maxVoltage = maxVoltage;
    _lowVoltage = lowVoltage;
    _minVoltage = minVoltage;
    
    if (!m_ina.setup(senseResistance, maxExpectedCurrent))
    {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Vrail Monitor Failed to initialize!");
        return;
    }
    
}

void INA_VRailMonitor::update(SensorStructs::INA_V_RAIL_t &data)
{
    m_ina.update();

    data.volt = static_cast<int>(m_ina.getBusV()*1e3); //convert to mV
    data.current = static_cast<int>(m_ina.getCurrent()*1e3); //convert to mA
    data.power = static_cast<int>(m_ina.getPower()*1e3); //convert to mW

    if ((data.volt < _lowVoltage) && !_lowVoltageTriggered)
    {
        RicCoreLogging::log<LOG_TARGET>( _name + ": low voltage, at " + std::to_string(data.volt) + "mV");
        _lowVoltageTriggered = true;
    }
    else if ((data.volt > _lowVoltage) && _lowVoltageTriggered)
    {
        _lowVoltageTriggered = false;
    }
    
    if (!_maxVoltage)
    {
        //cannot calculate if we dont have max voltage
        data.percent = 0;
    }
    else
    {
        data.percent = int(((float)(data.volt - _minVoltage) / (float)(_maxVoltage - _minVoltage)) * 100.0);
    }

}