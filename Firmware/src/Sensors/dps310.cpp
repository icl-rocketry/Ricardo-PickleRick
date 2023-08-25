#include "dps310.h"

#include <libriccore/riccorelogging.h>

#include <SPI.h>
#include <Dps3xx.h>
#include <math.h>
#include <array>
#include <Preferences.h>

#include "Config/types.h"
#include "Config/systemflags_config.h"

#include "sensorStructs.h"

DPS310::DPS310(SPIClass &spi, Types::CoreTypes::SystemStatus_t &systemstatus, uint8_t cs) : Dps3xx(),
                                                                                            _spi(spi),
                                                                                            _systemstatus(systemstatus),
                                                                                            _cs(cs),
                                                                                            _initialized(false)
{
}

void DPS310::setup()
{

    begin(_spi, _cs);

    int error = startMeasureBothCont(temp_mr, temp_osr, press_mr, press_osr);
    if (error)
    {
        _systemstatus.newFlag(SYSTEM_FLAG::ERROR_BARO, "DPS310 failed to start with code: " + std::to_string(error));
        return;
    }
    // start measure both cont enables FiFo, however we want to disable FiFo so that the result register
    // is always the most up to date reading and we dont particualrly care about historical readings.
    //  In reality we are likeyl polling this sensor alot quicker than it can update.
    if (disableFIFO())
    {
        _systemstatus.newFlag(SYSTEM_FLAG::ERROR_BARO, "DPS310 failed to disable FiFo!");
        return;
    }

    loadDPSCalibrationValues();

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("DPS310 Initialized!");
    _initialized = true;
}

void DPS310::update(SensorStructs::BARO_t &data)
{
    if (!_initialized)
    {
        return;
    }

    readDPS(data.press, data.temp);
    data.temp += 273.15;
    data.alt = toAltitude(data.press);
}

void DPS310::calibrateBaro()
{
    if (!_initialized)
    {
        return;
    }

    readDPS(refPress, refTemp);

    refTemp += 273.15; // convert from celcius to kelvin

    writeDPSCalibrationValues();
}

float DPS310::toAltitude(const float &pressure)
{

    constexpr float R = 287.052;     // specific gas constant R*/M0
    constexpr float g = 9.80665;     // standard gravity
    constexpr float t_grad = 0.0065; // gradient of temperature

    return refTemp / t_grad * (1 - exp((t_grad * R / g) * log(pressure / refPress)));
}

void DPS310::readDPS(float &pressure, float &temperature)
{
    constexpr int PSR_B2 = 0x00;
    constexpr int RW_BYTE = 0x80;

    constexpr size_t numbytes = 6;
    // this method assumes the dps310 is in background mode
    std::array<uint8_t, numbytes> buffer;

    // for some reason the dps requires a 'dummy' transaction. Probbaly due to the change of clock polarity messing clocking the data out or somethin like that
    // not really worth trying to fix any more than this. But if u are bored go for it. Maybe I should add a wasted hours counter on this
    _spi.beginTransaction(SPISettings(DPS3xx__SPI_MAX_FREQ,
                                      MSBFIRST,
                                      SPI_MODE3));
    _spi.transfer(PSR_B2 | RW_BYTE); // start at PSR_B2
    _spi.endTransaction();

    _spi.beginTransaction(SPISettings(DPS3xx__SPI_MAX_FREQ,
                                      MSBFIRST,
                                      SPI_MODE3));
    digitalWrite(_cs, LOW);
    _spi.transfer(PSR_B2 | RW_BYTE); // start at PSR_B2

    for (int i = 0; i < numbytes; i++)
    {
        buffer[i] = _spi.transfer(0); // read out the 6 result registers
    }

    digitalWrite(_cs, HIGH);
    _spi.endTransaction();

    int32_t raw_press = (uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2];
    int32_t raw_temp = (uint32_t)buffer[3] << 16 | (uint32_t)buffer[4] << 8 | (uint32_t)buffer[5];

    getTwosComplement(&raw_press, 24);
    getTwosComplement(&raw_temp, 24);

    temperature = calcTemp(raw_temp);
    pressure = calcPressure(raw_press);
}


void DPS310::writeDPSCalibrationValues(){
    Preferences pref;

    if (!pref.begin("DPS")){
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("nvs failed to start. Can't write DPS calibration values");
        return;
    }

    if (!pref.putFloat("TempRef", refTemp)){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("nvs error while writing");}
    if (!pref.putFloat("PressRef", refPress)){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("nvs error while writing");}
}


void DPS310::loadDPSCalibrationValues(){
    Preferences pref;

    if (!pref.begin("DPS", true)){
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("nvs failed to start");
        return;
    }

    refTemp = pref.getFloat("TempRef");
    refPress = pref.getFloat("PressRef");
}