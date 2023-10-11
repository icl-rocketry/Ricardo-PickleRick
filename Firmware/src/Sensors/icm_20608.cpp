#include "icm_20608.h"

#include <Arduino.h>
#include <SPI.h>
#include <string>

#include <libriccore/riccorelogging.h>

#include "Config/types.h"
#include "Config/systemflags_config.h"

#include <Preferences.h>

ICM_20608::ICM_20608(SPIClass& spi,Types::CoreTypes::SystemStatus_t& systemstatus,uint8_t cs):
_spi(spi),
_systemstatus(systemstatus),
_cs(cs), // update this with proper config value
_settings(8000000,MSBFIRST,SPI_MODE0)
{}

void ICM_20608::setup(const std::array<uint8_t,3>& axesOrder, const std::array<bool,3> axesFlip)
{
    writeRegister(PWR_MGMT_1, RESET);     // reset whole device
    delay(100);                            

    writeRegister(USER_CTRL, 0x00);       // disable fifo

    writeRegister(USER_CTRL, I2C_IF_DIS); // disable I2C mode as recommended in datasheet

    writeRegister(PWR_MGMT_1, CLK_ZGYRO); // set clock source
    delay(5);
    //check we are alive
    if (!alive()){
         _systemstatus.newFlag(SYSTEM_FLAG::ERROR_IMU, "Unable to initialize the icm 20608");
        return;
    }
    //set gyro and accel ranges -> update this later to process ranges provided
    //from config
    setRange(AccelRange::A_16_G,GyroRange::G_2000_DEGS); 

    writeRegister(PWR_MGMT_2,0x00); //switch everything on
    
    //check we are alive
    if (!alive()){
         _systemstatus.newFlag(SYSTEM_FLAG::ERROR_IMU, "Unable to initialize the icm 20608");
        return;
    }

    loadAccelGyroBias(); //load offsets from nvs

    axeshelper.setOrder(axesOrder);
    axeshelper.setFlip(axesFlip);

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("IMU Initialized");

}

bool ICM_20608::alive(){
    return (readRegister(WHO_AM_I) == WHO_AM_I_RES);
}

void ICM_20608::update(SensorStructs::ACCELGYRO_6AXIS_t& data)
{
  
    readAccel(data.ax,data.ay,data.az);
    readGyro(data.gx,data.gy,data.gz);
    readTemp(data.temp);

    if (calibrating) {
        calibrateBias();
    }

}

void ICM_20608::startCalibrateBias()
{

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Accel calibration started");

    measurements_made = 0;

    sum_gx = 0, sum_gy = 0, sum_gz = 0;
    sum_ax = 0, sum_ay = 0, sum_az = 0;

    calibrating = true;
}

void ICM_20608::calibrateBias()
{

    readGyroRaw(gx, gy, gz);
    readAccelRaw(ax, ay, az);

    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;

    sum_ax += ax;
    sum_ay += ay;
    sum_az += az;

    measurements_made += 1;

    if (measurements_made == number_measurements){

        offset_gx = -sum_gx / number_measurements;
        offset_gy = -sum_gy / number_measurements;
        offset_gz = -sum_gz / number_measurements;
        offset_ax = -sum_ax / number_measurements;
        offset_ay = -sum_ay / number_measurements;
        offset_az = 1 / accel_lsb_to_g - sum_az / number_measurements;
        writeAccelGyroBias();

        calibrating = false;

        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Accel calibration completed");

    }

}

void ICM_20608::setRange(AccelRange accel_range,GyroRange gyro_range)
{
    switch (gyro_range)
    {
    case G_250_DEGS:
        writeRegister(GYRO_CONFIG, DPS250);
        gyro_lsb_to_degs = 250.f / 32768.f;
        break;
    case G_500_DEGS:
        writeRegister(GYRO_CONFIG, DPS500);
        gyro_lsb_to_degs = 500.f / 32768.f;
        break;
    case G_1000_DEGS:
        writeRegister(GYRO_CONFIG, DPS1000);
        gyro_lsb_to_degs = 1000.f / 32768.f;
        break;
    case G_2000_DEGS:
        writeRegister(GYRO_CONFIG, DPS2000);
        gyro_lsb_to_degs = 2000.f / 32768.f;
        break;
    }

    switch (accel_range)
    {
    case A_2_G:
        writeRegister(ACCEL_CONFIG, G2);
        accel_lsb_to_g = 2.f / 32768.f;
        break;
    case A_4_G:
        writeRegister(ACCEL_CONFIG, G4);
        accel_lsb_to_g = 4.f / 32768.f;
        break;
    case A_8_G:
        writeRegister(ACCEL_CONFIG, G8);
        accel_lsb_to_g = 8.f / 32768.f;
        break;
    case A_16_G:
        writeRegister(ACCEL_CONFIG, G16);
        accel_lsb_to_g = 16.f / 32768.f;
        break;
    }
}

void ICM_20608::writeRegister(uint8_t reg, uint8_t val)
{
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);
    _spi.transfer(reg & ~(1 << 7)); // MSB = 0 for Writing
    _spi.transfer(val);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

uint8_t ICM_20608::readRegister(uint8_t reg)
{
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);
    _spi.transfer(reg | (1 << 7)); // MSB = 1 for Reading
    uint8_t val = _spi.transfer(0);
    delay(1);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
    return val;
}

void ICM_20608::readGyro(float &x, float &y, float &z)
{
    int16_t xi, yi, zi;

    readGyroRaw(xi, yi, zi);

    std::array<float, 3> gyro = axeshelper(std::array<float, 3>{(float)(xi + offset_gx) * gyro_lsb_to_degs,
                                                                (float)(yi + offset_gy) * gyro_lsb_to_degs,
                                                                 (float)(zi + offset_gz) * gyro_lsb_to_degs});

    x = gyro[0];
    y = gyro[1];
    z = gyro[2];
}

void ICM_20608::readAccel(float &x, float &y, float &z)
{
    int16_t xi, yi, zi;
    readAccelRaw(xi, yi, zi);
    std::array<float, 3> accel = axeshelper(std::array<float, 3>{(float)(xi + offset_ax) * accel_lsb_to_g,
                                                                 (float)(yi + offset_ay) * accel_lsb_to_g,
                                                                  (float)(zi + offset_az) * accel_lsb_to_g});

    x = accel[0];
    y = accel[1];
    z = accel[2];
}

void ICM_20608::readGyroRaw(int16_t &x, int16_t &y, int16_t &z)
{
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);

    _spi.transfer(GYRO_XOUT_H | (1 << 7));
    x = ((int16_t)_spi.transfer(GYRO_XOUT_L | (1 << 7))) << 8;
    x |= _spi.transfer(GYRO_YOUT_H | (1 << 7));
    y = ((int16_t)_spi.transfer(GYRO_YOUT_L | (1 << 7))) << 8;
    y |= _spi.transfer(GYRO_ZOUT_H | (1 << 7));
    z = ((int16_t)_spi.transfer(GYRO_ZOUT_L | (1 << 7))) << 8;
    z |= _spi.transfer(0);

    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

void ICM_20608::readAccelRaw(int16_t &x, int16_t &y, int16_t &z)
{
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);

    _spi.transfer(ACCEL_XOUT_H | (1 << 7));
    x = ((int16_t)_spi.transfer(ACCEL_XOUT_L | (1 << 7))) << 8;
    x |= _spi.transfer(ACCEL_YOUT_H | (1 << 7));
    y = ((int16_t)_spi.transfer(ACCEL_YOUT_L | (1 << 7))) << 8;
    y |= _spi.transfer(ACCEL_ZOUT_H | (1 << 7));
    z = ((int16_t)_spi.transfer(ACCEL_ZOUT_L | (1 << 7))) << 8;
    z |= _spi.transfer(0);

    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

void ICM_20608::readTempRaw(int16_t& temp)
{
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);

    _spi.transfer(TEMP_OUT_H | (1 << 7));
    temp = ((int16_t)_spi.transfer(TEMP_OUT_L | (1 << 7))) << 8;
    temp |= _spi.transfer(0x00);

    digitalWrite(_cs,HIGH);
    _spi.endTransaction();
}

void ICM_20608::readTemp(float& temp)
{
    int16_t temp_raw;
    readTempRaw(temp_raw);

    temp = (((float)temp_raw)/ temperature_sensitivity) + 25.0f;
}

void ICM_20608::writeAccelGyroBias(){
    Preferences pref;

    if (!pref.begin("IMU1")){
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("nvs failed to start. Can't write calbration offsets");
        return;
    }   
   
    if (!pref.putShort("gxBias",offset_gx)){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("nvs error while writing");};
    if (!pref.putShort("gyBias",offset_gy)){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("nvs error while writing");};
    if (!pref.putShort("gzBias",offset_gz)){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("nvs error while writing");};
    if (!pref.putShort("axBias",offset_ax)){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("nvs error while writing");};
    if (!pref.putShort("ayBias",offset_ay)){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("nvs error while writing");};
    if (!pref.putShort("azBias",offset_az)){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("nvs error while writing");};
    

}

void ICM_20608::loadAccelGyroBias(){
    Preferences pref;

    if (!pref.begin("IMU1",true)){
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("nvs failed to start");
        return;
    }

    offset_gx = pref.getShort("gxBias");
    offset_gy = pref.getShort("gyBias");
    offset_gz = pref.getShort("gzBias");
    offset_ax = pref.getShort("axBias");
    offset_ay = pref.getShort("ayBias");
    offset_az = pref.getShort("azBias");
}