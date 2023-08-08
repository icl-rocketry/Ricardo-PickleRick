#pragma once

#include <SparkFun_u-blox_GNSS_v3.h>
#include <Wire.h>

#include "Config/types.h"

#include "sensorStructs.h"

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

class Max_M10S
{
public:
    Max_M10S(TwoWire &wire, Types::CoreTypes::SystemStatus_t &systemstatus);
    void setup(const uint8_t address=0x42);
    void update(SensorStructs::GPS_t &gpsdata);

private:
    SFE_UBLOX_GNSS gnss;
    TwoWire &_wire; // pointer to wire object

    Types::CoreTypes::SystemStatus_t &_systemstatus; // pointer to system status object

    bool _i2cerror; // true if i2c failed to start

    
};
