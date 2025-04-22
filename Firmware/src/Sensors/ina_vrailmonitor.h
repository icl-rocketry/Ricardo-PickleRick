#pragma once

#include "sensorStructs.h"

#include <Wire.h>

#include <libriccore/riccorelogging.h>

#include "INA219.h"

class INA_VRailMonitor
{
public:
    /**
     * @brief Construct a new VRailMonitor object
     * 
     * @param vrail_name name of voltage rail for logging purposes
     * @param wire I2C wire to use
     * @param address I2C address of INA219
     */
    INA_VRailMonitor(std::string_view vrail_name,TwoWire &wire,uint8_t address);
    /**
     * @brief Set max, low and min voltage levels. Use max and min to accurately report battery percentage
     * 
     * @param maxVoltage maximum voltage in mV
     * @param lowVoltage low voltage in mV
     * @param minVoltage minium voltage in mV
     */
    void setup(int maxVoltage, int lowVoltage, int minVoltage);
    /**
     * @brief Read data into sensor struct
     * 
     * @param data 
     */
    void update(SensorStructs::INA_V_RAIL_t &data);

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
     * @brief INA219 object
     * 
     */
    INA219 m_ina;

    //! stolen striaght from flight and steel code 
    static constexpr float senseResistance =  2.0e-3; //ohms
    static constexpr float maxExpectedCurrent = 30; // 30A


    int _maxVoltage;
    int _lowVoltage;
    int _minVoltage;

    bool _lowVoltageTriggered;

};