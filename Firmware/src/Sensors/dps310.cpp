#include "dps310.h"


#include <libriccore/riccorelogging.h>

#include <SPI.h>
#include <Dps3xx.h>
#include <math.h>
#include <array>

#include "Config/types.h"
#include "Config/systemflags_config.h"


#include "sensorStructs.h"

DPS310::DPS310(SPIClass& spi, Types::CoreTypes::SystemStatus_t& systemstatus,uint8_t cs):
Dps3xx(),
_spi(spi),
_systemstatus(systemstatus),
_cs(cs),
_initialized(false)
{}

void DPS310::setup()
{

Dps3xx::begin(_spi,_cs);

int error  = Dps3xx::startMeasureBothCont(temp_mr,temp_osr,press_mr,press_osr);
if (error)
{
    _systemstatus.newFlag(SYSTEM_FLAG::ERROR_BARO,"DPS310 failed to start with code: " + std::to_string(error));
    return;
   
}
//start measure both cont enables FiFo, however we want to disable FiFo so that the result register 
//is always the most up to date reading and we dont particualrly care about historical readings. 
// In reality we are likeyl polling this sensor alot quicker than it can update.
if (DpsClass::disableFIFO())
{
    _systemstatus.newFlag(SYSTEM_FLAG::ERROR_BARO,"DPS310 failed to disable FiFo!");
    return;
}



RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("DPS310 Initialized!");
_initialized = true;

}

void DPS310::update(SensorStructs::BARO_t &data)
{
    if(!_initialized)
    {
        return;
    }

    readDPS(data.press,data.temp);
    readDPS(data.press,data.temp);
    data.temp += 273.15;
    data.alt = toAltitude(data.press);
}

void DPS310::calibrateBaro()
{
    if(!_initialized)
    {
        return;
    }

    readDPS(refPress,refTemp);
    refTemp += 273.15; //convert from celcius to kelvin
}


float DPS310::toAltitude(const float& pressure) {

    constexpr float R = 287.052; // specific gas constant R*/M0
    constexpr float g = 9.80665; // standard gravity 
    constexpr float t_grad = 0.0065; // gradient of temperature

    return refTemp / t_grad * (1 - exp((t_grad * R / g) * log(pressure / refPress)));
}

void DPS310::readDPS(float& pressure,float& temperature)
{
    constexpr int PSR_B2 = 0x00;
    constexpr int RW_BYTE = 0x80;

    constexpr size_t numbytes = 6;
    //this method assumes the dps310 is in background mode
    std::array<uint8_t,numbytes> buffer;

    _spi.beginTransaction(SPISettings(DPS3xx__SPI_MAX_FREQ,
                                           MSBFIRST,
                                           SPI_MODE3));


    digitalWrite(_cs,LOW);
    _spi.transfer(PSR_B2 | RW_BYTE); // start at PSR_B2
    for (int i = 0; i < numbytes; i++)
    {
        buffer[i] = _spi.transfer(0); //read out the 6 result registers
    }
    digitalWrite(_cs,HIGH);
    _spi.endTransaction();

    int32_t raw_press = (uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2];
    int32_t raw_temp = (uint32_t)buffer[3] << 16 | (uint32_t)buffer[4] << 8 | (uint32_t)buffer[5];

    getTwosComplement(&raw_press,24);
    getTwosComplement(&raw_temp,24);

    // getRawResult(&raw_temp,dps::registerBlocks[dps::RegisterBlocks_e::TEMP]);
    // getRawResult(&raw_press,dps::registerBlocks[dps::RegisterBlocks_e::PRS]);
    // getRawResult(&raw_temp,dps::registerBlocks[dps::RegisterBlocks_e::TEMP]);

    temperature = calcTemp(raw_temp);
    pressure = calcPressure(raw_press);

}