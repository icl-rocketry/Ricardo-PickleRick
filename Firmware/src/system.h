#pragma once

#include <libriccore/riccoresystem.h>

//config includes
#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"

#include <SPI.h>
#include <Wire.h>

#include "Commands/commands.h"

#include "Network/Interfaces/radio.h"
#include <libriccore/networkinterfaces/can/canbus.h>

#include "Sensors/sensors.h"
#include "Sensors/estimator.h"

#include "Sound/tunezHandler.h"

#include "Events/eventHandler.h"
#include "Deployment/deploymenthandler.h"
#include "Engine/enginehandler.h"
#include "Controller/controllerhandler.h"

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


    private:

        void setupSPI();
        void setupI2C();
        void setupPins();
        void loadComponentConfig();


};