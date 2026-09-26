#pragma once

#include <libriccore/riccoresystem.h>

#include <string_view>
#include <array>

//config includes
#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"
#include "Config/types.h"

#include <SPI.h>
#include <Wire.h>


#include <librrc/Remote/nrcremotepyro.h>

#include "Commands/commands.h"

#include <libriccore/networkinterfaces/can/canbus.h>

#include "Toaster/toaster.h"

#include "Sensors/estimator.h"

#include "Sound/tunezHandler.h"
#include "Storage/sdfat_store.h"

#include "ApogeeDetection/apogeedetect.h"

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

    CanBus<SYSTEM_FLAG> canbus;

    Sensors sensors;
    Estimator estimator;

    ApogeeDetect apogeedetect;

    TunezHandler tunezhandler;

    SdFat_Store primarysd;

    Toaster toaster;

private:

    void setupSPI();
    void setupI2C();
    void setupPins();
    void configureNetwork();
    void initializeLoggers();
    void logTelemetry();
    void loadConfig();

    static constexpr std::string_view log_path = "/Logs";
    static constexpr std::string_view config_path = "/Config/rml.jsonc";

    uint32_t telemetry_log_delta = 5000; //200hz
    uint32_t prev_telemetry_log_time;
};