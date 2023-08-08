#include "dps310.h"


#include <libriccore/riccorelogging.h>

#include <SPI.h>
#include <Dps3xx.h>
#include <math.h>

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

if (Dps3xx::startMeasureBothCont(temp_mr,temp_osr,press_mr,press_osr))
{
    _systemstatus.newFlag(SYSTEM_FLAG::ERROR_BARO,"DPS310 failed to start!");
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
    readDPS(data.press,data.temp);
    data.alt = toAltitude(data.press);
}

void DPS310::calibrateBaro()
{
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

}