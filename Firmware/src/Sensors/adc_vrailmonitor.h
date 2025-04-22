#pragma once

#include "sensorStructs.h"

#include <driver/gpio.h>
#include <driver/adc.h>
#include "esp_adc_cal.h"

#include <libriccore/riccorelogging.h>

class ADC_VRailMonitor
{
public:
    /**
     * @brief Construct a new VRailMonitor object
     * 
     * @param vrail_name name of voltage rail for logging purposes
     * @param pin pin to read voltage
     * @param r1 value of r1 in potential divider (unitless)
     * @param r2 value of r2 in potential divider (unitless)
     */
    ADC_VRailMonitor(std::string_view vrail_name,const uint8_t pin, const float r1,const float r2);
    /**
     * @brief Set max, low and min voltage levels. Use max and min to accurately report battery percentage
     * 
     * @param maxVoltage maximum voltage in mV
     * @param lowVoltage low voltage in mV
     * @param minVoltage minium voltage in mV
     */
    void setup(int maxVoltage, int lowVoltage,int minVoltage);
    /**
     * @brief Read data into sensor struct
     * 
     * @param data 
     */
    void update(SensorStructs::ADC_V_RAIL_t &data);

private:
    /**
     * @brief Alias for logging target
     * 
     */
    static constexpr auto LOG_TARGET = RicCoreLoggingConfig::LOGGERS::SYS;

    /**
     * @brief Name of voltage rail for logging purposes
     * 
     */
    const std::string _name;
    /**
     * @brief Pin to read voltage
     * 
     */
    const uint8_t _pin;

    /**
     * @brief Corresponding adc channel
     * 
     */
    adc_channel_t _channel;

    /**
     * @brief Corresonding adc unit
     * 
     */
    adc_unit_t _unit;

    /**
     * @brief Calibration of adc struct
     * 
     */
    esp_adc_cal_characteristics_t _adcCal;

    /**
     * @brief Flag for adc initialization
     * 
     */
    bool _adcInitialized;

    static constexpr adc_atten_t _atten = ADC_ATTEN_DB_11;
    static constexpr adc_bits_width_t _width = ADC_WIDTH_12Bit;


    /**
     * @brief esp32 pins can read upto 3.3v (3300mv) in 4095 steps, voltage divider halves the input voltage hence we get 2*3300 -> from actual board r1 = 9.22k, r2 = 8.79k
     *
     */
    const float factor;

    int _maxVoltage;
    int _lowVoltage;
    int _minVoltage;

    bool _lowVoltageTriggered;

    uint16_t sampleDelta = 20; // sample the voltage rail at 5hz
    uint32_t prevSampleTime = 0;
};

