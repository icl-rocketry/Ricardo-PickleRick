#include "Sensors/icm_20608.h"

ICM_20608::ICM_20608(SPIClass &spi, Types::CoreTypes::SystemStatus_t &systemstatus, uint8_t cs)
    : _spi(spi),
      _systemstatus(systemstatus),
      _cs(cs),  // update this with proper config value
      _settings(8000000, MSBFIRST, SPI_MODE0)
{
}

void ICM_20608::setup(const std::array<uint8_t, 3> &axesOrder, const std::array<bool, 3> axesFlip)
{
    writeRegister(PWR_MGMT_1, RESET);  // reset whole device
    delay(100);

    writeRegister(USER_CTRL, 0x00);  // disable fifo

    writeRegister(USER_CTRL, I2C_IF_DIS);  // disable I2C mode as recommended in datasheet

    writeRegister(PWR_MGMT_1, CLK_ZGYRO);  // set clock source
    delay(5);
    // check we are alive
    if (!alive())
    {
        _systemstatus.newFlag(SYSTEM_FLAG::ERROR_IMU, "Unable to initialize the icm 20608");
        return;
    }
    // set gyro and accel ranges -> update this later to process ranges provided
    // from config
    setRange(AccelRange::A_16_G, GyroRange::G_2000_DEGS);

    writeRegister(PWR_MGMT_2, 0x00);  // switch everything on

    // check we are alive
    if (!alive())
    {
        _systemstatus.newFlag(SYSTEM_FLAG::ERROR_IMU, "Unable to initialize the icm 20608");
        return;
    }

    axeshelper.setOrder(axesOrder);
    axeshelper.setFlip(axesFlip);

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("IMU Initialized");
}

void ICM_20608::update(SensorStructs::ACCELGYRO_6AXIS_t &data)
{
    readAccel(data.ax, data.ay, data.az);
    readGyro(data.gx, data.gy, data.gz);
    readTemp(data.temp);
    
}

bool ICM_20608::alive() { return (readRegister(WHO_AM_I) == WHO_AM_I_RES); }

void ICM_20608::setRange(AccelRange accel_range, GyroRange gyro_range)
{
    switch (gyro_range)
    {
        case G_250_DEGS:
            writeRegister(GYRO_CONFIG, DPS250);
            gyro_lsb_to_rads = (250.f / 32768.f) * (M_PI / 180.0f);
            break;
        case G_500_DEGS:
            writeRegister(GYRO_CONFIG, DPS500);
            gyro_lsb_to_rads = (500.f / 32768.f) * (M_PI / 180.0f);
            break;
        case G_1000_DEGS:
            writeRegister(GYRO_CONFIG, DPS1000);
            gyro_lsb_to_rads = (1000.f / 32768.f) * (M_PI / 180.0f);
            break;
        case G_2000_DEGS:
            writeRegister(GYRO_CONFIG, DPS2000);
            gyro_lsb_to_rads = (2000.f / 32768.f) * (M_PI / 180.0f);
            break;
    }

    switch (accel_range)
    {
        case A_2_G:
            writeRegister(ACCEL_CONFIG, G2);
            accel_lsb_to_ms2 = 2.f * g / 32768.f;
            break;
        case A_4_G:
            writeRegister(ACCEL_CONFIG, G4);
            accel_lsb_to_ms2 = 4. * g / 32768.f;
            break;
        case A_8_G:
            writeRegister(ACCEL_CONFIG, G8);
            accel_lsb_to_ms2 = 8.f * g / 32768.f;
            break;
        case A_16_G:
            writeRegister(ACCEL_CONFIG, G16);
            accel_lsb_to_ms2 = 16.f * g / 32768.f;
            break;
    }
}

void ICM_20608::readGyro(float &x, float &y, float &z)
{
    int16_t xi, yi, zi;
    readGyroRaw(xi, yi, zi);

    std::array<float, 3> gyro = axeshelper(std::array<float, 3>{
        (float)(xi) * gyro_lsb_to_rads, 
        (float)(yi) * gyro_lsb_to_rads,
        (float)(zi) * gyro_lsb_to_rads
    });

    x = gyro[0];
    y = gyro[1];
    z = gyro[2];
}

void ICM_20608::readAccel(float &x, float &y, float &z)
{
    int16_t xi, yi, zi;
    readAccelRaw(xi, yi, zi);

    std::array<float, 3> accel = axeshelper(std::array<float, 3>{
        (float)(xi) * accel_lsb_to_ms2, 
        (float)(yi) * accel_lsb_to_ms2,
        (float)(zi) * accel_lsb_to_ms2
    });

    x = accel[0];
    y = accel[1];
    z = accel[2];
}

void ICM_20608::readTemp(float &temp)
{
    int16_t temp_raw;
    readTempRaw(temp_raw);

    temp = (((float)temp_raw) / temperature_sensitivity) + 25.0f;
}

void ICM_20608::readGyroRaw(int16_t &x, int16_t &y, int16_t &z)
{
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);

    _spi.transfer(GYRO_XOUT_H | (1 << 7));  // address only
    x = ((int16_t)_spi.transfer(0x00)) << 8;
    x |= _spi.transfer(0x00);
    y = ((int16_t)_spi.transfer(0x00)) << 8;
    y |= _spi.transfer(0x00);
    z = ((int16_t)_spi.transfer(0x00)) << 8;
    z |= _spi.transfer(0x00);

    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

void ICM_20608::readAccelRaw(int16_t &x, int16_t &y, int16_t &z)
{
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);

    _spi.transfer(ACCEL_XOUT_H | (1 << 7));  // address only
    x = ((int16_t)_spi.transfer(0x00)) << 8;
    x |= _spi.transfer(0x00);
    y = ((int16_t)_spi.transfer(0x00)) << 8;
    y |= _spi.transfer(0x00);
    z = ((int16_t)_spi.transfer(0x00)) << 8;
    z |= _spi.transfer(0x00);

    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

void ICM_20608::readTempRaw(int16_t &temp)
{
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);

    _spi.transfer(TEMP_OUT_H | (1 << 7));
    temp = ((int16_t)_spi.transfer(TEMP_OUT_L | (1 << 7))) << 8;
    temp |= _spi.transfer(0x00);

    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

uint8_t ICM_20608::readRegister(uint8_t reg)
{
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);
    _spi.transfer(reg | (1 << 7));  // MSB = 1 for Reading
    uint8_t val = _spi.transfer(0);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
    return val;
}

void ICM_20608::writeRegister(uint8_t reg, uint8_t val)
{
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);
    _spi.transfer(reg & ~(1 << 7));  // MSB = 0 for Writing
    _spi.transfer(val);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}
