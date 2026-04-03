#include "Sensors/dps368.h"


DPS368::DPS368(SPIClass& spi, Types::CoreTypes::SystemStatus_t& systemstatus, uint8_t cs)
    : _spi(spi),
      _systemstatus(systemstatus),
      _cs(cs),
      _settings(8000000, MSBFIRST, SPI_MODE3)  // DPS368 SPI mode '11' (CPOL=CPHA=1)
{
}

void DPS368::setup()
{
    // Soft reset
    writeRegister(RESET_REG, SOFT_RST_VAL);
    delay(40);  // wait for coefficients to become available (40 ms per datasheet)

    // Check product ID
    if (!alive())
    {
        _systemstatus.newFlag(SYSTEM_FLAG::ERROR_BARO, "Unable to initialize DPS368");
        return;
    }

    // Poll until sensor and coefficients are ready
    uint32_t timeout = millis() + 100;
    while ((readRegister(MEAS_CFG) & (COEF_RDY | SENSOR_RDY)) != (COEF_RDY | SENSOR_RDY))
    {
        if (millis() > timeout)
        {
            _systemstatus.newFlag(SYSTEM_FLAG::ERROR_BARO, "DPS368 timed out waiting for ready");
            return;
        }
        delay(1);
    }

    readCalibrationCoefficients();

    // Read which sensor the calibration coefficients are based on and mirror it
    // into TMP_CFG so temperature compensation is accurate (datasheet section 8.12)
    uint8_t coef_src = (readRegister(COEF_SRCE) & 0x80);  // bit 7: 0=ASIC, 1=MEMS
    uint8_t tmp_ext  = coef_src ? 0x80 : 0x00;

    // Pressure: 1 meas/sec, 16x oversampling (standard precision)
    writeRegister(PRS_CFG, (PM_RATE_1HZ << 4) | PM_PRC_16X);

    // Temperature: 1 meas/sec, 1x oversampling, same source as coefficients
    writeRegister(TMP_CFG, tmp_ext | (0x00 << 4) | TMP_PRC_1X);

    // Enable result bit-shift for pressure (required when oversampling > 8x)
    writeRegister(CFG_REG, P_SHIFT_EN);

    // Apply errata fix for fuse bit issue on some DPS368 units (undocumented registers)
    writeRegister(0x0E, 0xA5);
    writeRegister(0x0F, 0x96);
    writeRegister(0x62, 0x02);
    writeRegister(0x0E, 0x00);
    writeRegister(0x0F, 0x00);

    // Take a blocking temperature measurement to seed compensation before continuous mode
    writeRegister(MEAS_CFG, MEAS_CMD_TEMP);
    delay(4);  // 1x oversampling measurement time ~3.6 ms
    int32_t rawT;
    readTemperatureRaw(rawT);
    _lastTraw_sc = static_cast<float>(rawT) / kT;

    // Start continuous pressure + temperature measurements
    writeRegister(MEAS_CFG, MEAS_CONT_PT);

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Barometer Initialized");
}

void DPS368::update(SensorStructs::BARO_t& data)
{
    int32_t rawP, rawT;
    readTemperatureRaw(rawT);
    readPressureRaw(rawP);

    if (rawT != 0)
    {
        _lastTraw_sc = static_cast<float>(rawT) / kT;
    }

    data.temp  = compensateTemperature(rawT);
    data.press = compensatePressure(rawP, _lastTraw_sc);
    
}

// -----------------------------------------------------------------------------
// Private
// -----------------------------------------------------------------------------

bool DPS368::alive()
{
    // Lower nibble of register 0x0D is the product ID (0x0 = DPS368)
    return ((readRegister(PROD_ID) & 0x0F) == 0x00);
}

void DPS368::readCalibrationCoefficients()
{
    uint8_t buffer[18];

    uint8_t address = 0x10 & ~0x80;

    readRegisters(address, buffer, 18);

    // compose coefficients from buffer content
    _c0Half = ((uint32_t)buffer[0] << 4) | (((uint32_t)buffer[1] >> 4) & 0x0F);
    getTwosComplement(&_c0Half, 12);
    // c0 is only used as c0*0.5, so c0_half is calculated immediately
    _c0Half = _c0Half / 2U;

    // now do the same thing for all other coefficients
    _c1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
    getTwosComplement(&_c1, 12);
    _c00 = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | (((uint32_t)buffer[5] >> 4) & 0x0F);
    getTwosComplement(&_c00, 20);
    _c10 = (((uint32_t)buffer[5] & 0x0F) << 16) | ((uint32_t)buffer[6] << 8) | (uint32_t)buffer[7];
    getTwosComplement(&_c10, 20);

    _c01 = ((uint32_t)buffer[8] << 8) | (uint32_t)buffer[9];
    getTwosComplement(&_c01, 16);

    _c11 = ((uint32_t)buffer[10] << 8) | (uint32_t)buffer[11];
    getTwosComplement(&_c11, 16);
    _c20 = ((uint32_t)buffer[12] << 8) | (uint32_t)buffer[13];
    getTwosComplement(&_c20, 16);
    _c21 = ((uint32_t)buffer[14] << 8) | (uint32_t)buffer[15];
    getTwosComplement(&_c21, 16);
    _c30 = ((uint32_t)buffer[16] << 8) | (uint32_t)buffer[17];
    getTwosComplement(&_c30, 16);

}

void DPS368::readPressureRaw(int32_t& raw)
{
    uint8_t buf[3];
    readRegisters(PSR_B2, buf, 3);

    raw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    // Sign-extend from 24-bit 2's complement
    if (raw > 8388607) raw -= 16777216;
}

void DPS368::readTemperatureRaw(int32_t& raw)
{
    uint8_t buf[3];
    readRegisters(TMP_B2, buf, 3);

    raw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    // Sign-extend from 24-bit 2's complement
    if (raw > 8388607) raw -= 16777216;
}

float DPS368::compensateTemperature(int32_t raw)
{
    // Tcomp (°C) = c0 * 0.5 + c1 * Traw_sc  (datasheet section 4.9.2)
    float Traw_sc = static_cast<float>(raw) / kT;
    return (_c0Half) + (_c1 * Traw_sc);
}

float DPS368::compensatePressure(int32_t rawP, float Traw_sc)
{
    // Pcomp (Pa) = c00 + Praw_sc*(c10 + Praw_sc*(c20 + Praw_sc*c30))
    //            + Traw_sc*c01 + Traw_sc*Praw_sc*(c11 + Praw_sc*c21)
    // (datasheet section 4.9.1)
    float Praw_sc = static_cast<float>(rawP) / kP;

    return static_cast<float>(_c00)
         + Praw_sc * (static_cast<float>(_c10)
         + Praw_sc * (static_cast<float>(_c20)
         + Praw_sc *  static_cast<float>(_c30)))
         + Traw_sc * static_cast<float>(_c01)
         + Traw_sc * Praw_sc * (static_cast<float>(_c11)
         + Praw_sc * static_cast<float>(_c21));
}

// -----------------------------------------------------------------------------
// SPI primitives
// -----------------------------------------------------------------------------

uint8_t DPS368::readRegister(uint8_t reg)
{
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);
    _spi.transfer(reg | 0x80);  // bit7 = 1 → read
    uint8_t val = _spi.transfer(0x00);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
    return val;
}

void DPS368::writeRegister(uint8_t reg, uint8_t val)
{
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);
    _spi.transfer(reg & 0x7F);  // bit7 = 0 → write
    _spi.transfer(val);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

void DPS368::readRegisters(uint8_t reg, uint8_t* buf, uint8_t len)
{
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);
    _spi.transfer(reg | 0x80);  // read, auto-increment
    for (uint8_t i = 0; i < len; ++i)
    {
        buf[i] = _spi.transfer(0xFF);
    }
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

// -----------------------------------------------------------------------------
// helpers
// -----------------------------------------------------------------------------

void DPS368::getTwosComplement(int32_t *raw, uint8_t length)
{
    if (*raw & ((uint32_t)1 << (length - 1)))
    {
        *raw -= (uint32_t)1 << length;
    }
}