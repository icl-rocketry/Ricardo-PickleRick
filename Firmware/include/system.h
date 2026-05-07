#pragma once

#include <SPI.h>
#include <Wire.h>
#include <memory>
#include <string_view>
#include <array>
#include <ArduinoJson.h>

#include <librrc/Remote/nrcremotepyro.h>
#include <libriccore/riccoresystem.h>
#include <libriccore/networkinterfaces/can/canbus.h>
#include <libriccore/storage/wrappedfile.h>

#include "Commands/commands.h"
#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/types.h"
#include "Config/general_config.h"
#include "Config/services_config.h"
#include "Config/timing_config.h"
#include "GNC/GNCController.h"
#include "PowerMonitor/PowerMonitor.h"
#include "Estimator/estimator.h"
#include "Loggers/EstimatorLogger/estimatorlogframe.h"
#include "Loggers/TelemetryLogger/telemetrylogframe.h"
#include "Network/Interfaces/radio.h"
#include "Sensors/sensors.h"
#include "States/preflight.h"
#include "Storage/sdfat_file.h"
#include "Storage/sdfat_store.h"

class System : public RicCoreSystem<System,SYSTEM_FLAG,Commands::ID>
{
    public:

        System();
        void systemSetup();
        void systemUpdate();

        //board communication
        SPIClass vspi;
        SPIClass hspi;
        TwoWire I2C;

        Radio radio;
        CanBus<SYSTEM_FLAG> canbus;

        Sensors sensors;
        Estimator estimator;

        SdFat_Store primarysd;

        GNCController controller;

        PowerMonitor powermonitor;
    private:

        void setupSPI();
        void setupI2C();
        void setupPins();
        void configureNetwork();
        void loadConfig();
        void initializeLoggers();
        void logEstimator();
        void logTelemetry();
        void configureRadio(JsonObjectConst conf);
        static bool timerDue(uint32_t current_time, uint32_t& prev_time, uint32_t delta);
        void updateSlowSensors(uint32_t current_time);
        void reportPerformance();

        static constexpr std::string_view log_path = "/Logs";
        static constexpr std::string_view config_path = "/Config/rml.jsonc";
        
        uint32_t telemetry_log_delta = TimingConfig::Scheduler::TELEMETRY_LOG_DELTA_US;
        uint32_t prev_telemetry_log_time = 0;
        uint32_t estimator_update_delta = TimingConfig::Scheduler::ESTIMATOR_UPDATE_DELTA_US;
        uint32_t prev_estimator_update_time = 0;
        uint32_t gps_update_delta = TimingConfig::Sensors::GPS_READ_DELTA_US;
        uint32_t prev_gps_update_time = 0;
        uint32_t baro_update_delta = TimingConfig::Sensors::BARO_READ_DELTA_US;
        uint32_t prev_baro_update_time = 0;
        uint32_t mag_update_delta = TimingConfig::Sensors::MAG_READ_DELTA_US;
        uint32_t prev_mag_update_time = 0;
        uint32_t rail_update_delta = TimingConfig::Sensors::RAIL_READ_DELTA_US;
        uint32_t prev_rail_update_time = 0;
        uint32_t lidar_update_delta = TimingConfig::Sensors::LIDAR_READ_DELTA_US;
        uint32_t prev_lidar_update_time = 0;
        uint32_t power_monitor_update_delta = TimingConfig::Scheduler::POWER_MONITOR_UPDATE_DELTA_US;
        uint32_t prev_power_monitor_update_time = 0;
        uint32_t estimator_log_delta = TimingConfig::Scheduler::ESTIMATOR_LOG_DELTA_US;
        uint32_t prev_estimator_log_time = 0;

        uint32_t perf_report_time = 0;
        uint32_t fast_path_count = 0;
        uint32_t slow_sensor_count = 0;
        uint32_t power_monitor_count = 0;
        uint32_t log_path_count = 0;
        uint64_t fast_path_time_us = 0;
        uint64_t sensor_fast_time_us = 0;
        uint64_t estimator_time_us = 0;
        uint64_t slow_sensor_time_us = 0;
        uint64_t power_monitor_time_us = 0;
        uint64_t log_path_time_us = 0;
        uint32_t max_fast_path_time_us = 0;
        uint32_t max_sensor_fast_time_us = 0;
        uint32_t max_estimator_time_us = 0;
        uint32_t max_slow_sensor_time_us = 0;
        uint32_t max_power_monitor_time_us = 0;
        uint32_t max_log_path_time_us = 0;

        uint32_t gps_update_count = 0;
        uint32_t baro_update_count = 0;
        uint32_t mag_update_count = 0;
        uint32_t rail_update_count = 0;
        uint32_t lidar_update_count = 0;
        uint64_t gps_update_time_us = 0;
        uint64_t baro_update_time_us = 0;
        uint64_t mag_update_time_us = 0;
        uint64_t rail_update_time_us = 0;
        uint64_t lidar_update_time_us = 0;
        uint32_t max_gps_update_time_us = 0;
        uint32_t max_baro_update_time_us = 0;
        uint32_t max_mag_update_time_us = 0;
        uint32_t max_rail_update_time_us = 0;
        uint32_t max_lidar_update_time_us = 0;

};
