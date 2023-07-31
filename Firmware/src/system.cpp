#include "system.h"

#include <memory>

#include <libriccore/riccoresystem.h>

#include <HardwareSerial.h>

#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"
#include "Config/general_config.h"
#include "Config/services_config.h"

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

#include "States/preflight.h"

#include <ArduinoJson.h>

System::System() : RicCoreSystem(Commands::command_map, Commands::defaultEnabledCommands, Serial),
                   vspi(VSPI),
                   hspi(HSPI), // CHANGE FOR esp32s3
                   I2C(0),
                   radio(hspi,  PinMap::LoraCs, PinMap::LoraReset, -1, systemstatus, RADIO_MODE::TURN_TIMEOUT,  2),
                   canbus(systemstatus, PinMap::TxCan, PinMap::RxCan, 3),
                   sensors(hspi, I2C, systemstatus),
                   estimator(systemstatus),
                   deploymenthandler(networkmanager, static_cast<uint8_t>(Services::ID::DeploymentHandler), I2C),
                   enginehandler(networkmanager, static_cast<uint8_t>(Services::ID::EngineHandler)),
                   controllerhandler(enginehandler),
                   eventhandler(enginehandler, deploymenthandler),
                   apogeedetect(20){};

void System::systemSetup()
{

    Serial.setRxBufferSize(GeneralConfig::SerialRxSize);
    Serial.begin(GeneralConfig::SerialBaud);

    // intialize rnp message logger
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::SYS>().initialize(networkmanager);

    setupPins();
    // intialize i2c interface
    setupI2C();
    // initalize spi interface
    setupSPI();

    tunezhandler.setup();
    // network interfaces
    radio.setup();
    canbus.setup();

    // add interfaces to netmanager
    networkmanager.addInterface(&radio);
    networkmanager.addInterface(&canbus);

    networkmanager.enableAutoRouteGen(false);
    networkmanager.setNoRouteAction(NOROUTE_ACTION::DUMP, {});

    loadComponentConfig();

    // sensors.setup(configcontroller.get()["Sensors"]);
    JsonObject dummy;
    sensors.setup(dummy);
    estimator.setup();

    // initialize statemachine with preflight state
    statemachine.initalize(std::make_unique<Preflight>(*this));
};

void System::systemUpdate()
{
    tunezhandler.update();
    sensors.update();
    estimator.update(sensors.getData());
};

void System::setupSPI()
{
    vspi.begin();
    vspi.setFrequency(1000000);
    vspi.setBitOrder(MSBFIRST);
    vspi.setDataMode(SPI_MODE0);

    hspi.begin();
    hspi.setFrequency(8000000);
    hspi.setBitOrder(MSBFIRST);
    hspi.setDataMode(SPI_MODE0);
}

void System::setupI2C()
{
    I2C.begin(PinMap::_SDA, PinMap::_SCL, GeneralConfig::I2C_FREQUENCY);
}

void System::setupPins()
{
    pinMode(PinMap::LoraCs, OUTPUT);
    pinMode(PinMap::ImuCs_1, OUTPUT);
    pinMode(PinMap::ImuCs_2, OUTPUT);
    pinMode(PinMap::BaroCs, OUTPUT);
    pinMode(PinMap::MagCs, OUTPUT);
    pinMode(PinMap::SdCs_1, OUTPUT);
    pinMode(PinMap::SdCs_2, OUTPUT);

    // initialise cs pins
    digitalWrite(PinMap::LoraCs, HIGH);
    digitalWrite(PinMap::ImuCs_1, HIGH);
    digitalWrite(PinMap::ImuCs_2, HIGH);
    digitalWrite(PinMap::BaroCs, HIGH);
    digitalWrite(PinMap::MagCs, HIGH);
    digitalWrite(PinMap::SdCs_1, HIGH);
    digitalWrite(PinMap::SdCs_2, HIGH);
}

void System::loadComponentConfig()
{
    JsonObject dummy;
    deploymenthandler.setup(dummy["t"]);
    enginehandler.setup(dummy["t"]);
    controllerhandler.setup(dummy["t"]);
    eventhandler.setup(dummy["t"]);
    //      // create config controller object
    //   ConfigController configcontroller(&storagecontroller,&logcontroller);
    //   configcontroller.load(); // load configuration from sd card into ram

    //   //enumerate deployers engines controllers and events from config file
    //   try
    //   {
    //     deploymenthandler.setup(configcontroller.get()["Deployers"]);
    //     enginehandler.setup(configcontroller.get()["Engines"]);
    //     controllerhandler.setup(configcontroller.get()["Controllers"]);
    //     eventhandler.setup(configcontroller.get()["Events"]);
    //   }
    //   catch (const std::exception& e)
    //   {
    //     Serial.println("exception:");
    //     Serial.println(std::string(e.what()).c_str());
    //     //impelment panic handler to send crashed message to gc
    //     throw std::runtime_error("broke");
    //   }
    //   //register deployment and engine handler services
    networkmanager.registerService(static_cast<uint8_t>(Services::ID::DeploymentHandler), deploymenthandler.getThisNetworkCallback());
    networkmanager.registerService(static_cast<uint8_t>(Services::ID::EngineHandler), enginehandler.getThisNetworkCallback());
}