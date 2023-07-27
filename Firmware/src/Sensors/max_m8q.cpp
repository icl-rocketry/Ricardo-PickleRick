#include "max_m8q.h"



#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>



#include <libriccore/riccorelogging.h>

#include "Config/types.h"
#include "Config/systemflags_config.h"

#include "sensorStructs.h"



Max_M8Q::Max_M8Q(TwoWire& wire, Types::CoreTypes::SystemStatus_t& systemstatus) :
    gnss(),
    _wire(wire),
    _systemstatus(systemstatus),
{}

void Max_M8Q::setup()
{
    if(!gnss.begin(_wire)){
        _systemstatus.newFlag(SYSTEM_FLAG::ERROR_GPS,"GPS I2C not found at address");
        _i2cerror = true;
    }else{
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Max_M8Q Initialized");   
        _i2cerror = false;
    }
    //turn off nmea messaging
    gnss.setI2COutput(COM_TYPE_UBX);
    if (!gnss.setDynamicModel(DYN_MODEL_AIRBORNE4g)){
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("GPS failed to set dynamics model");
    } 
    gnss.setNavigationFrequency(10); //Set output to 10 times a second
    gnss.setAutoPVT(true);

}

void Max_M8Q::update(SensorStructs::GPS_t& gpsdata)
{
    if (_i2cerror){
        return;
    }

   if (gnss.getPVT() && (!gnss.getInvalidLlh())){ // check if new navigation solution is available
        gpsdata.updated = true;

       gpsdata.lat = (float)gnss.getLatitude()/10000000.0; //degrees
       gpsdata.lng = (float)gnss.getLongitude()/10000000.0; // degrees
       gpsdata.alt = gnss.getAltitude(); //mm above ellipsoid

       gpsdata.pdop = gnss.getPDOP();

       gpsdata.v_n = gnss.getNedNorthVel(); // mm/s
       gpsdata.v_e = gnss.getNedEastVel(); // mm/s
       gpsdata.v_d = gnss.getNedDownVel(); // mm/s

       gpsdata.sat = gnss.getSIV(); 
       gpsdata.fix = gnss.getFixType();
       gpsdata.valid = gnss.getGnssFixOk();

       gpsdata.year = gnss.getYear();
       gpsdata.month = gnss.getMonth();
       gpsdata.day = gnss.getDay();
       gpsdata.hour = gnss.getHour();
       gpsdata.minute = gnss.getMinute();
       gpsdata.second = gnss.getSecond();

       if (gpsdata.fix < 3){
           if (!_systemstatus.flagSetOr(SYSTEM_FLAG::ERROR_GPS)){
               _systemstatus.newFlag(SYSTEM_FLAG::ERROR_GPS,"GPS bad fix");
           }
       }else{
           if(_systemstatus.flagSetOr(SYSTEM_FLAG::ERROR_GPS)){
               _systemstatus.deleteFlag(SYSTEM_FLAG::ERROR_GPS);
           }
       }

   }else{
       gpsdata.updated = false;
   }
    
}