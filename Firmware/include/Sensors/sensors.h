#pragma once


#include <SPI.h>
#include <Wire.h>
#include <memory>
#include <functional>
#include <ArduinoJson.h>

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>
#include <librrc/Helpers/jsonconfighelper.h>

#include "Config/types.h"
#include "Config/systemflags_config.h"
#include "Config/pinmap_config.h"
#include "Sensors/packets/hitlpacket.h"
#include "Sensors/max_m10s.h"
#include "Sensors/dps368.h"
#include "Sensors/icm_20608.h"
#include "Sensors/h3lis331dl.h"
#include "Sensors/mmc5983ma.h"
#include "Sensors/adc_vrailmonitor.h"
#include "Sensors/ina_vrailmonitor.h"
#include "Sensors/sensorStructs.h"

class Sensors
{
public:
    Sensors(SPIClass &spi, TwoWire &I2C, Types::CoreTypes::SystemStatus_t &systemstatus);

    void setup(JsonObjectConst config);
    void update();

    /**
     * @brief Get the Raw Sensor Data
     *
     * @return const SensorStructs::raw_measurements_t&
     */
    const SensorStructs::raw_measurements_t &getData();

    // Sensor Calibration Functions
    void calibrateMag(MagCalibrationParameters magcal);

    std::function<void(std::unique_ptr<RnpPacketSerialized>)> getHitlCallback();

private:
    SensorStructs::raw_measurements_t sensors_raw;
    Types::CoreTypes::SystemStatus_t& _systemstatus;

    MAX_M10S gps;
    DPS368 baro;
    ICM_20608 accelgyro;
    H3LIS331DL accel;
    MMC5983MA mag;
    ADC_VRailMonitor logicrail;
    INA_VRailMonitor deprail;


    /**
     * @brief Handle fake sensor data packets from hardware in the loop service
     * 
     * @param packet_ptr 
     */
    void hitlHandler(std::unique_ptr<RnpPacketSerialized> packet_ptr);
    void hitlCommandHandler(RnpPacketSerialized& packet);
    bool _hitlEnabled;

    void hitlUpdateSensorError(uint8_t sensor_state,SYSTEM_FLAG flag);

};
