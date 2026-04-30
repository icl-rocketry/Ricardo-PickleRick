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
#include "GNC/GNCController.h"
#include "PowerMonitor/PowerMonitor.h"
#include "Estimator/estimator.h"
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
        void logTelemetry();
        void configureRadio(JsonObjectConst conf);

        static constexpr std::string_view log_path = "/Logs";
        static constexpr std::string_view config_path = "/Config/rml.jsonc";
        
        uint32_t telemetry_log_delta = 10000; // 100Hz
        uint32_t prev_telemetry_log_time;

};