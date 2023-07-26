#include "VRailMonitor.h"

#include <string>

#include <libriccore/riccorelogging.h>

#include "sensorStructs.h"


VRailMonitor::VRailMonitor(std::string_view vrail_name,const uint8_t pin, const float r1,const float r2):
_name(vrail_name),
_pin(pin),
factor(((r1+r2)/r2) * (3300.f / 4095.f)),
_maxVoltage(0),
_lowVoltage(0)
{};

void VRailMonitor::setup(uint16_t maxVoltage, uint16_t lowVoltage,uint16_t minVoltage){
    _maxVoltage = maxVoltage;
    _lowVoltage = lowVoltage;
    _minVoltage = minVoltage;
}

void VRailMonitor::update(SensorStructs::V_RAIL_t &data)
{

    if (millis() - prevSampleTime >= sampleDelta)
    {

        const uint16_t reading = analogRead(_pin);
        data.volt = (uint16_t)(factor * (float)reading); // voltage in mV

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
            data.percent = uint16_t(((float)(data.volt - _minVoltage) / (float)(_maxVoltage - _minVoltage)) * 100.0);
        }
    }
}
