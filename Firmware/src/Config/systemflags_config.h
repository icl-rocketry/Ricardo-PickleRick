#pragma once

#include <stdint.h>
#include <type_traits>

enum class SYSTEM_FLAG:uint32_t{
    //state flags
    STATE_PRE_FLIGHT = (1 << 0),
    STATE_FLIGHT = (1 << 1),
    STATE_LANDING = (1 << 2), 
    STATE_POST_FLIGHT = (1 << 3),
    STATE_HARD_ABORT = (1 << 4),
    STATE_SOFT_ABORT = (1 << 5),

    //flags
    DEBUG = (1 << 7),
    //critical messages 
    ERROR_SPI = (1 << 8),
    ERROR_I2C = (1 << 9),
    ERROR_SERIAL = (1 << 10),
    ERROR_LORA = (1 << 11),
    ERROR_BARO = (1 << 12),
    ERROR_BATT = (1 << 13),
    ERROR_GPS = (1 << 14),
    ERROR_IMU = (1 << 15),
    ERROR_HACCEL = (1 << 16),
    ERROR_MAG = (1 << 17),
    ERROR_ESTIMATOR = (1 << 18),
    ERROR_SD = (1 << 19),
    ERROR_FLASH = (1 << 20),
    ERROR_CAN = (1 << 21),
    ERROR_FLIGHTCHECK = (1<<22),
    //if rocket is inverted
    ERROR_ORIENTATION = (1 << 23),
    //warn
    WARN_BATT = (1 << 24),
    //FLIGHTPHASES 
    FLIGHTPHASE_BOOST = (1 << 25),
    
    FLIGHTPHASE_COAST = (1 << 26),
    FLIGHTPHASE_APOGEE = (1 << 27)
    
};

using system_flag_t = uint32_t;

