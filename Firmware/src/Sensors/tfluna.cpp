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
    const uint8_t status = _wire.endTransmission();
    if (status != 0)
    {
        _systemstatus.newFlag(SYSTEM_FLAG::ERROR_LIDAR, "TF-Luna not found at 0x10");
        RicCoreLogging::log<LOG_TARGET>("TF-Luna init failed");
        Serial.printf("TF-Luna init failed: addr=0x%02X i2c_status=%u\n", _address, status);
        return;
    }
    _setup_ok = true;
    RicCoreLogging::log<LOG_TARGET>("TF-Luna initialized");
    Serial.printf("TF-Luna initialized: addr=0x%02X\n", _address);
}

void TFLuna::update(SensorStructs::LIDAR_t& data)
{
    // Read 6 bytes: DIST_LOW, DIST_HIGH, AMP_LOW, AMP_HIGH, TEMP_LOW, TEMP_HIGH
    uint8_t buf[6];
    if (!readRegisters(REG_DIST_LOW, buf, 6))
    {
        data.valid = false;
        _setup_ok = false;
        _read_fail_count++;

        const uint32_t now_ms = millis();
        if (now_ms - _last_read_fail_log_ms >= 1000)
        {
            _last_read_fail_log_ms = now_ms;
            RicCoreLogging::log<LOG_TARGET>("TF-Luna read failed");
            Serial.printf("TF-Luna read failed: addr=0x%02X count=%lu\n",
                          _address,
                          static_cast<unsigned long>(_read_fail_count));
        }
        return;
    }

    if (!_setup_ok)
    {
        _setup_ok = true;
        RicCoreLogging::log<LOG_TARGET>("TF-Luna read recovered");
        Serial.printf("TF-Luna read recovered: addr=0x%02X\n", _address);
    }

    data.dist  = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
    data.amp   = static_cast<uint16_t>(buf[2]) | (static_cast<uint16_t>(buf[3]) << 8);

    // Temperature unit from register is 0.01 °C
    uint16_t rawTemp = static_cast<uint16_t>(buf[4]) | (static_cast<uint16_t>(buf[5]) << 8);
    data.temp  = static_cast<float>(rawTemp) * 0.01f;

    data.valid = (data.amp >= AMP_MIN) && (data.amp != AMP_OVEREXPOSURE);
    data.timestamp_us = micros();

    const uint32_t now_ms = millis();
    if (now_ms - _last_read_ok_log_ms >= 1000)
    {
        _last_read_ok_log_ms = now_ms;
        Serial.printf("TF-Luna read ok: dist=%u amp=%u temp=%.2f valid=%u raw=%02X %02X %02X %02X %02X %02X\n",
                      data.dist,
                      data.amp,
                      data.temp,
                      data.valid ? 1 : 0,
                      buf[0],
                      buf[1],
                      buf[2],
                      buf[3],
                      buf[4],
                      buf[5]);
    }
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
