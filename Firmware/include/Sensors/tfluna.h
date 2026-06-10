#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <cstdint>

#include <libriccore/riccorelogging.h>

#include "Config/types.h"
#include "Config/systemflags_config.h"
#include "Sensors/sensorStructs.h"

class TFLuna
{
public:
    TFLuna(TwoWire& wire,
           Types::CoreTypes::SystemStatus_t& systemstatus,
           uint8_t address = TF_LUNA_I2C_ADDR);

    void setup();
    void update(SensorStructs::LIDAR_t& data);

private:
    bool readRegisters(uint8_t reg, uint8_t* buf, uint8_t len);

    TwoWire&                          _wire;
    Types::CoreTypes::SystemStatus_t& _systemstatus;
    uint8_t                           _address;
    bool                              _setup_ok{false};
    uint32_t                          _read_fail_count{0};
    uint32_t                          _last_read_fail_log_ms{0};

    static constexpr uint8_t  TF_LUNA_I2C_ADDR = 0x10;

    // Register map (Appendix III of TF-Luna instruction manual)
    static constexpr uint8_t REG_DIST_LOW  = 0x00;
    static constexpr uint8_t REG_AMP_LOW   = 0x02;
    static constexpr uint8_t REG_TEMP_LOW  = 0x04;

    static constexpr uint16_t AMP_MIN      = 100;
    static constexpr uint16_t AMP_OVEREXPOSURE = 0xFFFF;

    static constexpr auto LOG_TARGET = RicCoreLoggingConfig::LOGGERS::SYS;
};
