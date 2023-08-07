#pragma once

#include "sensorStructs.h"

#include <libriccore/riccorelogging.h>

class VRailMonitor
{
public:
    /**
     * @brief Construct a new VRailMonitor object
     * 
     * @param systemstatus reference to system status object
     * @param pin pin to read voltage
     * @param r1 value of r1 in potential divider (unitless)
     * @param r2 value of r2 in potential divider (unitless)
     */
    VRailMonitor(std::string_view vrail_name,const uint8_t pin, const float r1,const float r2);
    /**
     * @brief Set max, low and min voltage levels. Use max and min to accurately report battery percentage
     * 
     * @param maxVoltage maximum voltage in mV
     * @param lowVoltage low voltage in mV
     * @param minVoltage minium voltage in mV
     */
    void setup(uint16_t maxVoltage, uint16_t lowVoltage, uint16_t minVoltage);
    /**
     * @brief Read data into sensor struct
     * 
     * @param data 
     */
    void update(SensorStructs::V_RAIL_t &data);

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
     * @brief esp32 pins can read upto 3.3v (3300mv) in 4095 steps, voltage divider halves the input voltage hence we get 2*3300 -> from actual board r1 = 9.22k, r2 = 8.79k
     *
     */
    const float factor;

    uint16_t _maxVoltage;
    uint16_t _lowVoltage;
    uint16_t _minVoltage;

    bool _lowVoltageTriggered;

    uint16_t sampleDelta = 20; // sample the voltage rail at 5hz
    uint32_t prevSampleTime = 0;
};

