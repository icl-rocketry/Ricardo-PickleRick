#include "VRailMonitor.h"

#include <string>

#include <driver/gpio.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp32-hal-adc.h>

#include <libriccore/riccorelogging.h>

#include "sensorStructs.h"


VRailMonitor::VRailMonitor(std::string_view vrail_name,const uint8_t pin, const float r1,const float r2):
    _name(vrail_name),
    _pin(pin),
    _channel(ADC_CHANNEL_0),//default
    _unit(ADC_UNIT_1),
    _adcCal(),
    _adcInitialized(false),
    factor(((r1+r2)/r2)),
    _maxVoltage(0),
    _lowVoltage(0),
    _minVoltage(0)
{};

void VRailMonitor::setup(uint16_t maxVoltage, uint16_t lowVoltage,uint16_t minVoltage){
    _maxVoltage = maxVoltage;
    _lowVoltage = lowVoltage;
    _minVoltage = minVoltage;

    int error = 0;
    //get channel and adc unit
    int8_t channel = digitalPinToAnalogChannel(_pin);
    if (channel > (SOC_ADC_MAX_CHANNEL_NUM - 1))
    {
        _channel = static_cast<adc_channel_t>(channel-SOC_ADC_MAX_CHANNEL_NUM);
        _unit = ADC_UNIT_2;
        
        error += adc2_config_channel_atten(static_cast<adc2_channel_t>(_channel),_atten);
    }
    else
    {
        _channel = static_cast<adc_channel_t>(channel);
        _unit = ADC_UNIT_1;
        error += adc1_config_width(_width);
        error += adc1_config_channel_atten(static_cast<adc1_channel_t>(_channel),_atten);
    }

    if (error)
    {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Vrail Monitor Failed to initialize!");
        return;
    }

    //characerise ADC
    esp_adc_cal_characterize(_unit,_atten,_width,1100,&_adcCal);

    _adcInitialized = true;
    
}

void VRailMonitor::update(SensorStructs::V_RAIL_t &data)
{
    if(!_adcInitialized)
    {
        return;
    }

    if (millis() - prevSampleTime >= sampleDelta)
    {   
        int raw_reading;
        if (_unit == ADC_UNIT_1)
        {
            raw_reading = adc1_get_raw(static_cast<adc1_channel_t>(_channel));
        }
        else
        {
            
             adc2_get_raw(static_cast<adc2_channel_t>(_channel),_width,&raw_reading);
        }

        uint32_t reading = esp_adc_cal_raw_to_voltage(raw_reading,&_adcCal);

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


