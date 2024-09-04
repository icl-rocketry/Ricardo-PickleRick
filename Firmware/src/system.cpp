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

#include "Network/Interfaces/radio.h"
#include <libriccore/networkinterfaces/can/canbus.h>

#include "Sensors/sensors.h"
#include "Sensors/estimator.h"
#include "Sensors/sensorStructs.h"

#include "Sound/tunezHandler.h"

#include "Events/eventHandler.h"
#include "Deployment/deploymenthandler.h"
#include "Deployment/PCA9534.h"
#include "Deployment/PCA9534Gpio.h"
#include "Engine/enginehandler.h"
#include "Controller/controllerhandler.h"
#include "Storage/sdfat_store.h"
#include "Storage/sdfat_file.h"
#include "Loggers/TelemetryLogger/telemetrylogframe.h"

#include "States/preflight.h"

#include "hal/usb_serial_jtag_ll.h"




#ifdef CONFIG_IDF_TARGET_ESP32S3
static constexpr int VSPI_BUS_NUM = 0;
static constexpr int HSPI_BUS_NUM = 1;
#else
static constexpr int VSPI_BUS_NUM = VSPI;
static constexpr int HSPI_BUS_NUM = HSPI;
#endif

System::System() : RicCoreSystem(Commands::command_map, Commands::defaultEnabledCommands, Serial),
                   vspi(VSPI_BUS_NUM),
                   hspi(HSPI_BUS_NUM),
                   I2C(0),
                   radio(hspi,  PinMap::LoraCs, PinMap::LoraReset, -1, systemstatus, RADIO_MODE::TURN_TIMEOUT, 2),
                   canbus(systemstatus, PinMap::TxCan, PinMap::RxCan, 3),
                   sensors(hspi, I2C, systemstatus),
                   estimator(systemstatus),
                   deploymenthandler(networkmanager, localPyroMap, localServoMap, static_cast<uint8_t>(Services::ID::DeploymentHandler)),
                   enginehandler(networkmanager, localPyroMap, localServoMap, static_cast<uint8_t>(Services::ID::EngineHandler)),
                   controllerhandler(enginehandler),
                   eventhandler(enginehandler, deploymenthandler, localPyroMap, localServoMap),
                   apogeedetect(20),
                   primarysd(vspi,PinMap::SdCs_1,SD_SCK_MHZ(20),false,&systemstatus),
                   pyroPinExpander0(0x20,I2C),
                   pyro0(PCA9534Gpio(pyroPinExpander0,PinMap::Ch0Fire),PCA9534Gpio(pyroPinExpander0,PinMap::Ch0Cont),networkmanager),
                   pyro1(PCA9534Gpio(pyroPinExpander0,PinMap::Ch1Fire),PCA9534Gpio(pyroPinExpander0,PinMap::Ch1Cont),networkmanager),
                   pyro2(PCA9534Gpio(pyroPinExpander0,PinMap::Ch2Fire),PCA9534Gpio(pyroPinExpander0,PinMap::Ch2Cont),networkmanager),
                   pyro3(PCA9534Gpio(pyroPinExpander0,PinMap::Ch3Fire),PCA9534Gpio(pyroPinExpander0,PinMap::Ch3Cont),networkmanager),
                   pwmPinExpander0(0x40,I2C,50),
                   servo0(PCA9685PWM(PinMap::servo0pin,pwmPinExpander0),networkmanager,"srv0"),
                   servo1(PCA9685PWM(PinMap::servo1pin,pwmPinExpander0),networkmanager,"srv1"),
                   servo2(PCA9685PWM(PinMap::servo2pin,pwmPinExpander0),networkmanager,"srv2"),
                   servo3(PCA9685PWM(PinMap::servo3pin,pwmPinExpander0),networkmanager,"srv3")
                   {};

void System::systemSetup()
{

    Serial.setRxBufferSize(GeneralConfig::SerialRxSize);
    Serial.begin(GeneralConfig::SerialBaud);
  

    setupPins();
    // intialize i2c interface
    setupI2C();
    // initalize spi interface
    setupSPI();

    primarysd.setup();

    initializeLoggers();    

    tunezhandler.setup();
    // network interfaces
    radio.setup();
    canbus.setup();

    // add interfaces to netmanager
    configureNetwork();

    //register pryo services
    setupLocalPyros();
    //register servo serv ices
    setupLocalServos();

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

void System::setupLocalPyros()
{
    if (pyroPinExpander0.setup())
    {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("I2C pyro pin expander alive");

        pyro0.setup();
        pyro1.setup();
        pyro2.setup();
        pyro3.setup();
        
        networkmanager.registerService(static_cast<uint8_t>(Services::ID::Pyro0),pyro0.getThisNetworkCallback());
        networkmanager.registerService(static_cast<uint8_t>(Services::ID::Pyro1),pyro1.getThisNetworkCallback());
        networkmanager.registerService(static_cast<uint8_t>(Services::ID::Pyro2),pyro2.getThisNetworkCallback());
        networkmanager.registerService(static_cast<uint8_t>(Services::ID::Pyro3),pyro3.getThisNetworkCallback());
    }
    else
    {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("I2C pyro pin expander failed to respond");
    }

};

void System::setupLocalServos()
{
    if (pwmPinExpander0.setup())
    {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("I2C pwm expander alive");

        servo0.setup();
        servo1.setup();
        servo2.setup();
        servo3.setup();
        
        networkmanager.registerService(static_cast<uint8_t>(Services::ID::Servo0),servo0.getThisNetworkCallback());
        networkmanager.registerService(static_cast<uint8_t>(Services::ID::Servo1),servo1.getThisNetworkCallback());
        networkmanager.registerService(static_cast<uint8_t>(Services::ID::Servo2),servo2.getThisNetworkCallback());
        networkmanager.registerService(static_cast<uint8_t>(Services::ID::Servo3),servo3.getThisNetworkCallback());
    }
    else
    {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("I2C pwm expander failed to respond");
    }

};

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

        primarysd.mkdir("/Config"); // ensure config directory exists

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
        configureRadio(configDoc.as<JsonObjectConst>()["Radio"]);
        estimator.configure(configDoc.as<JsonObjectConst>()["Estimator"]);

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
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("SD Init Failed");
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

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("SD Init Complete");
}

void System::logTelemetry()
{
    if (micros() - prev_telemetry_log_time > telemetry_log_delta)
    {
        // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(std::to_string(uxTaskGetStackHighWaterMark(primarysd.getHandle())));
        
        // std::string logstring = "int:" + std::to_string(usb_serial_jtag_ll_get_intsts_mask());
        // std::stringstream s;
        // s << std::hex << Serial.getRxQueue() <<"\n";
        // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("sd card state: " + std::to_string(primarysd.getError()));

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

        const RadioInterfaceInfo* radio_info = reinterpret_cast<const RadioInterfaceInfo*>(radio.getInfo());

        logframe.rssi = radio_info->rssi;
        logframe.packet_rssi = radio_info->packet_rssi;
        logframe.snr = radio_info->snr;
        logframe.packet_snr = radio_info->packet_snr;

        logframe.timestamp = micros();

        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::TELEMETRY>(logframe);

        prev_telemetry_log_time = micros();
    }
}

void System::configureNetwork()
{   
    networkmanager.setNodeType(NODETYPE::HUB);
    networkmanager.addInterface(&radio);
    networkmanager.addInterface(&canbus);

    networkmanager.enableAutoRouteGen(true);
    networkmanager.setNoRouteAction(NOROUTE_ACTION::BROADCAST, {1,2});

    // RoutingTable flightRouting;
    // flightRouting.setRoute((uint8_t)DEFAULT_ADDRESS::GROUNDSTATION_GATEWAY,Route{2,1,{}});
    // flightRouting.setRoute((uint8_t)DEFAULT_ADDRESS::GROUNDSTATION,Route{2,2,{}});
    // flightRouting.setRoute(17,Route{3,2,{}});
    // flightRouting.setRoute(18,Route{3,2,{}});
    // flightRouting.setRoute(5,Route{3,2,{}});
    // flightRouting.setRoute(6,Route{3,2,{}});
    // flightRouting.setRoute(7,Route{3,2,{}});
    // flightRouting.setRoute(8,Route{3,2,{}});
    // flightRouting.setRoute(9,Route{3,2,{}});
    // flightRouting.setRoute(10,Route{3,2,{}});
    // flightRouting.setRoute(11,Route{3,2,{}});
    // flightRouting.setRoute(12,Route{3,2,{}});
    // flightRouting.setRoute(13,Route{3,2,{}});
    // flightRouting.setRoute(14,Route{3,2,{}});
    // flightRouting.setRoute(15,Route{3,2,{}});
    // flightRouting.setRoute(16,Route{3,2,{}});
    // flightRouting.setRoute(50,Route{3,2,{}});
    // flightRouting.setRoute(51,Route{3,2,{}});
    // flightRouting.setRoute(52,Route{3,2,{}});
    // flightRouting.setRoute(100,Route{3,2,{}});
    // flightRouting.setRoute(101,Route{3,2,{}});
    // flightRouting.setRoute(102,Route{3,2,{}});
    // flightRouting.setRoute(150,Route{2,2,{}});

    
    // networkmanager.setRoutingTable(flightRouting);
    // networkmanager.updateBaseTable(); // save the new base table

};

void System::configureRadio(JsonObjectConst conf)
{
    using namespace LIBRRC::JsonConfigHelper;

    RadioConfig radioConfig = radio.getConfig(); // get default config
    try
    {
        bool override = getIfContains<bool>(conf,"Override",false);

        radioConfig.frequency = getIfContains<long>(conf,"Frequency",radioConfig.frequency);
        radioConfig.sync_byte = getIfContains<int>(conf,"SyncByte",radioConfig.sync_byte); // default 0xf3
        radioConfig.bandwidth = getIfContains<long>(conf,"Bandwidth",radioConfig.bandwidth);
        radioConfig.spreading_factor = getIfContains<int>(conf,"SpreadingFactor",radioConfig.spreading_factor);
        radioConfig.txPower = getIfContains<int>(conf,"TxPower",radioConfig.txPower);
        radio.setConfig(radioConfig,override);
    }
    catch (const std::exception &e)
    {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Exception occured while loading flight config! - " + std::string(e.what()));
        return;
    }
}