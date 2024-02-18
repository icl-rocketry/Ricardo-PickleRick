#include "system.h"

#include <memory>

#include <ArduinoJson.h>

#include <libriccore/riccoresystem.h>
#include <libriccore/storage/wrappedfile.h>


#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"
#include "Config/general_config.h"
#include "Config/services_config.h"

#include "Commands/commands.h"

#include "Network/Interfaces/tdma.h"
#include <libriccore/networkinterfaces/can/canbus.h>

#include "Sensors/sensors.h"
#include "Sensors/estimator.h"
#include "Sensors/sensorStructs.h"

#include "Sound/tunezHandler.h"

#include "Events/eventHandler.h"
#include "Deployment/deploymenthandler.h"
#include "Engine/enginehandler.h"
#include "Controller/controllerhandler.h"
#include "Storage/sdfat_store.h"
#include "Storage/sdfat_file.h"
#include "Loggers/TelemetryLogger/telemetrylogframe.h"

#include "States/preflight.h"



#ifdef CONFIG_IDF_TARGET_ESP32S3
static constexpr int VSPI_BUS_NUM = 0;
static constexpr int HSPI_BUS_NUM = 1;
#else
static constexpr int VSPI_BUS_NUM = VSPI;
static constexpr int HSPI_BUS_NUM = HSPI;
#endif

System::System() : RicCoreSystem(Commands::command_map, Commands::defaultEnabledCommands, Serial),
                   vspi(VSPI_BUS_NUM),
                   hspi(HSPI_BUS_NUM), // CHANGE FOR esp32s3
                   I2C(0),
                   tdma(hspi,  PinMap::LoraCs, PinMap::LoraReset, -1, systemstatus, networkmanager, 2, "TDMA Radio"),
                   canbus(systemstatus, PinMap::TxCan, PinMap::RxCan, 3),
                   sensors(hspi, I2C, systemstatus),
                   estimator(systemstatus),
                   deploymenthandler(networkmanager, static_cast<uint8_t>(Services::ID::DeploymentHandler), I2C),
                   enginehandler(networkmanager, static_cast<uint8_t>(Services::ID::EngineHandler)),
                   controllerhandler(enginehandler),
                   eventhandler(enginehandler, deploymenthandler),
                   apogeedetect(20),
                   primarysd(vspi,PinMap::SdCs_1,SD_SCK_MHZ(20),false,&systemstatus)
                   {};

void System::systemSetup()
{

    Serial.setRxBufferSize(GeneralConfig::SerialRxSize);
    Serial.begin(GeneralConfig::SerialBaud);
    delay(3000);

    setupPins();
    // intialize i2c interface
    setupI2C();
    // initalize spi interface
    setupSPI();

    primarysd.setup();

    initializeLoggers();

    tunezhandler.setup();
    // network interfaces
    tdma.setup();
    canbus.setup();

    // add interfaces to netmanager
    networkmanager.setNodeType(NODETYPE::HUB);
    networkmanager.addInterface(&tdma);
    networkmanager.addInterface(&canbus);

    networkmanager.enableAutoRouteGen(true);
    networkmanager.setNoRouteAction(NOROUTE_ACTION::BROADCAST, {1,2,3});


    loadConfig();

    estimator.setup();

    // initialize statemachine with preflight state
    statemachine.initalize(std::make_unique<Preflight>(*this));
};

void System::systemUpdate()
{
    tunezhandler.update();
    sensors.update();
    estimator.update(sensors.getData());
    logTelemetry();
};

void System::setupSPI()
{
    vspi.begin(PinMap::V_SCLK,PinMap::V_MISO,PinMap::V_MOSI);
    vspi.setFrequency(1000000);
    vspi.setBitOrder(MSBFIRST);
    vspi.setDataMode(SPI_MODE0);

    hspi.begin(PinMap::H_SCLK,PinMap::H_MISO,PinMap::H_MOSI);
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

void System::loadConfig()
{
    DynamicJsonDocument configDoc(16384); //allocate 16kb for config doc MAXSIZE
    DeserializationError jsonError;
    // get wrapped file for config doc -> returns nullptr if cant open
    

    //only try load file if sd card is present
    if (primarysd.getState() == StoreBase::STATE::NOMINAL)
    {

        primarysd.mkdir("/Config");

        std::unique_ptr<WrappedFile> config_file_ptr = primarysd.open(config_path,FILE_MODE::READ);

        if (config_file_ptr != nullptr)
        {
            //cast non-owning wrapped file ptr to sdfat_wrappedfile ptr
            SdFat_WrappedFile* sdfat_wrapped_file_ptr = reinterpret_cast<SdFat_WrappedFile*>(config_file_ptr.get());
            //lock the file store device lock
            {
            RicCoreThread::ScopedLock sl(sdfat_wrapped_file_ptr->getDevLock());
            jsonError = deserializeJson(configDoc,sdfat_wrapped_file_ptr->IStream());

            }
        }
        else
        {
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Error opening config file!");
        }

        if (jsonError)
        {
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Error deserializing JSON - " + std::string(jsonError.c_str()));
        }
    }
    
    //enumerate deployers engines controllers and events from config file
    try
    {
        sensors.setup(configDoc.as<JsonObjectConst>()["Sensors"]);

        deploymenthandler.setup(configDoc.as<JsonObjectConst>()["Deployers"]);
        enginehandler.setup(configDoc.as<JsonObjectConst>()["Engines"]);
        controllerhandler.setup(configDoc.as<JsonObjectConst>()["Controllers"]);
        eventhandler.setup(configDoc.as<JsonObjectConst>()["Events"]);

    }
    catch (const std::exception &e)
    {
         RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Exception occured while loading flight config! - " + std::string(e.what()));

         throw e; //continue throwing as we dont want to continue
    }
   
    //   //register deployment and engine handler services
    networkmanager.registerService(static_cast<uint8_t>(Services::ID::DeploymentHandler), deploymenthandler.getThisNetworkCallback());
    networkmanager.registerService(static_cast<uint8_t>(Services::ID::EngineHandler), enginehandler.getThisNetworkCallback());
}

void System::initializeLoggers()
{   
    //check if sd card is mounted
    if (primarysd.getState() != StoreBase::STATE::NOMINAL)
    {
        loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::SYS>().initialize(nullptr,networkmanager);

        return;
    }

    //open log files
    //get unique directory for logs
    std::string log_directory_path = primarysd.generateUniquePath(log_path,"");
    //make new directory
    primarysd.mkdir(log_directory_path);

    std::unique_ptr<WrappedFile> syslogfile = primarysd.open(log_directory_path + "/syslog.txt",static_cast<FILE_MODE>(O_WRITE | O_CREAT | O_AT_END));
    std::unique_ptr<WrappedFile> telemetrylogfile = primarysd.open(log_directory_path + "/telemetrylog.txt",static_cast<FILE_MODE>(O_WRITE | O_CREAT | O_AT_END),50); 
    
    // intialize sys logger
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::SYS>().initialize(std::move(syslogfile),networkmanager);
   
    //initialize telemetry logger
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::TELEMETRY>().initialize(std::move(telemetrylogfile));

}

void System::logTelemetry()
{
    if (micros() - prev_telemetry_log_time > telemetry_log_delta)
    {
        const SensorStructs::raw_measurements_t& raw_sensors = sensors.getData();
        const SensorStructs::state_t& estimator_state =  estimator.getData();
        TelemetryLogframe logframe;
        
        logframe.gps_long = raw_sensors.gps.lng;
        logframe.gps_lat = raw_sensors.gps.lat;
        logframe.gps_alt = raw_sensors.gps.alt;
        logframe.gps_v_n = raw_sensors.gps.v_n;
        logframe.gps_v_e = raw_sensors.gps.v_e;
        logframe.gps_v_d = raw_sensors.gps.v_d;
        logframe.gps_sat = raw_sensors.gps.sat;
        logframe.gps_fix = raw_sensors.gps.fix;
        logframe.ax = raw_sensors.accelgyro.ax;
        logframe.ay = raw_sensors.accelgyro.ay;
        logframe.az = raw_sensors.accelgyro.az;
        logframe.h_ax = raw_sensors.accel.ax;
        logframe.h_ay = raw_sensors.accel.ay;
        logframe.h_az = raw_sensors.accel.az;
        logframe.gx = raw_sensors.accelgyro.gx;
        logframe.gy = raw_sensors.accelgyro.gy;
        logframe.gz = raw_sensors.accelgyro.gz;
        logframe.mx = raw_sensors.mag.mx;
        logframe.my = raw_sensors.mag.my;
        logframe.mz = raw_sensors.mag.mz;
        logframe.imu_temp = raw_sensors.accelgyro.temp;
        logframe.baro_alt = raw_sensors.baro.alt;
        logframe.baro_temp = raw_sensors.baro.temp;
        logframe.baro_press = raw_sensors.baro.press;
        logframe.batt_volt = raw_sensors.logicrail.volt;
        logframe.batt_percent = raw_sensors.logicrail.percent;
        logframe.roll = estimator_state.eulerAngles[0];
        logframe.pitch = estimator_state.eulerAngles[1];
        logframe.yaw = estimator_state.eulerAngles[2];
        logframe.q0 = estimator_state.orientation.w();
        logframe.q1 = estimator_state.orientation.x();
        logframe.q2 = estimator_state.orientation.y();
        logframe.q3 = estimator_state.orientation.z();
        logframe.pn = estimator_state.position[0];
        logframe.pe = estimator_state.position[1];
        logframe.pd = estimator_state.position[2];
        logframe.vn = estimator_state.velocity[0];
        logframe.ve = estimator_state.velocity[1];
        logframe.vd = estimator_state.velocity[2];
        logframe.an = estimator_state.acceleration[0];
        logframe.ae = estimator_state.acceleration[1];
        logframe.ad = estimator_state.acceleration[2];

        const RadioInterfaceInfo* radio_info = reinterpret_cast<const RadioInterfaceInfo*>(tdma.getInfo());

        logframe.rssi = radio_info->rssi;
        logframe.snr = radio_info->snr;

        logframe.timestamp = micros();

        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::TELEMETRY>(logframe);

        prev_telemetry_log_time = micros();
    }
}

