#include "Sensors/tfluna.h"

TFLuna::TFLuna(TwoWire& wire,
               Types::CoreTypes::SystemStatus_t& systemstatus,
               uint8_t address)
    : _wire(wire),
      _systemstatus(systemstatus),
      _address(address)
{}

void TFLuna::setup()
{
    _wire.beginTransmission(_address);
    if (_wire.endTransmission() != 0)
    {
        _systemstatus.newFlag(SYSTEM_FLAG::ERROR_LIDAR, "TF-Luna not found at 0x10");
        RicCoreLogging::log<LOG_TARGET>("TF-Luna init failed");
        return;
    }
    RicCoreLogging::log<LOG_TARGET>("TF-Luna initialized");
}

void TFLuna::update(SensorStructs::LIDAR_t& data)
{
    // Read 6 bytes: DIST_LOW, DIST_HIGH, AMP_LOW, AMP_HIGH, TEMP_LOW, TEMP_HIGH
    uint8_t buf[6];
    if (!readRegisters(REG_DIST_LOW, buf, 6))
    {
        return;
    }

    data.dist  = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
    data.amp   = static_cast<uint16_t>(buf[2]) | (static_cast<uint16_t>(buf[3]) << 8);

    // Temperature unit from register is 0.01 °C
    uint16_t rawTemp = static_cast<uint16_t>(buf[4]) | (static_cast<uint16_t>(buf[5]) << 8);
    data.temp  = static_cast<float>(rawTemp) * 0.01f;

    data.valid = (data.amp >= AMP_MIN) && (data.amp != AMP_OVEREXPOSURE);
}

bool TFLuna::readRegisters(uint8_t reg, uint8_t* buf, uint8_t len)
{
    _wire.beginTransmission(_address);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0)
    {
        return false;
    }
    if (_wire.requestFrom(_address, len) != len)
    {
        return false;
    }
    for (uint8_t i = 0; i < len; i++)
    {
        buf[i] = _wire.read();
    }
    return true;
}
