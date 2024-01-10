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

#include "Network/Interfaces/radio.h"
#include <libriccore/networkinterfaces/can/canbus.h>

#include "Sensors/sensors.h"
#include "Sensors/estimator.h"

#include "Sound/tunezHandler.h"

#include "Events/eventHandler.h"
#include "Deployment/deploymenthandler.h"
#include "Deployment/PCA9534.h"
#include "Deployment/PCA9534Gpio.h"
#include "Engine/enginehandler.h"
#include "Controller/controllerhandler.h"
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

        Radio radio;
        CanBus<SYSTEM_FLAG> canbus;

        Sensors sensors;
        Estimator estimator;

        DeploymentHandler deploymenthandler;
        EngineHandler enginehandler;

        ControllerHandler controllerhandler;
        EventHandler eventhandler;

        ApogeeDetect apogeedetect;
        
        TunezHandler tunezhandler;

        SdFat_Store primarysd;

    private:

        void setupSPI();
        void setupI2C();
        void setupPins();
        void configureNetwork();
        void setupLocalPyros();
        void loadConfig();
        void initializeLoggers();
        void logTelemetry();

        static constexpr std::string_view log_path = "/Logs";
        static constexpr std::string_view config_path = "/Config/rml.jsonc";
        

        uint32_t telemetry_log_delta = 10000;
        uint32_t prev_telemetry_log_time;
        
        PCA9534 pyroPinExpander0;

        Types::LocalPyro_t pyro0;
        Types::LocalPyro_t pyro1;
        Types::LocalPyro_t pyro2;
        Types::LocalPyro_t pyro3;
        const std::array<Types::LocalPyro_t*,4> localPyroMap = {&pyro0,&pyro1,&pyro2,&pyro3};
        


};