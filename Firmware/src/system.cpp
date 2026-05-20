#include "system.h"

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
                   radio(hspi,  PinMap::LoraCs, PinMap::LoraReset, PinMap::LoraInt, systemstatus, RADIO_MODE::TURN_TIMEOUT, 2),
                   canbus(systemstatus, PinMap::TxCan, PinMap::RxCan, 3),
                   sensors(hspi, I2C, systemstatus),
                   estimator(systemstatus),
                   primarysd(vspi,PinMap::SdCs_1,SD_SCK_MHZ(20),false,&systemstatus),
                   controller("controller", Services::ID::Controller, networkmanager),
                   powermonitor("powermonitor",static_cast<uint8_t>(Services::ID::PowerMonitor), networkmanager)
                   {};

bool System::timerDue(const uint32_t current_time, uint32_t& prev_time, const uint32_t delta)
{
    if (prev_time == 0)
    {
        prev_time = current_time;
        return false;
    }

    if (current_time - prev_time < delta)
    {
        return false;
    }

    prev_time += delta;
    if (current_time - prev_time >= delta)
    {
        prev_time = current_time;
    }

    return true;
}

void System::updateSlowSensors(const uint32_t current_time)
{
    const auto recordSensorTiming = [](const uint32_t dt,
                                       uint32_t& count,
                                       uint64_t& total_time,
                                       uint32_t& max_time)
    {
        count++;
        total_time += dt;
        if (dt > max_time)
        {
            max_time = dt;
        }
    };

    if (timerDue(current_time, prev_gps_update_time, gps_update_delta))
    {
        const uint32_t start = micros();
        sensors.updateGps();
        recordSensorTiming(micros() - start, gps_update_count, gps_update_time_us, max_gps_update_time_us);
    }
    if (timerDue(current_time, prev_baro_update_time, baro_update_delta))
    {
        const uint32_t start = micros();
        sensors.updateBaro();
        recordSensorTiming(micros() - start, baro_update_count, baro_update_time_us, max_baro_update_time_us);
    }
    if (timerDue(current_time, prev_mag_update_time, mag_update_delta))
    {
        const uint32_t start = micros();
        sensors.updateMag();
        recordSensorTiming(micros() - start, mag_update_count, mag_update_time_us, max_mag_update_time_us);
    }
    if (timerDue(current_time, prev_rail_update_time, rail_update_delta))
    {
        const uint32_t start = micros();
        sensors.updateRails();
        recordSensorTiming(micros() - start, rail_update_count, rail_update_time_us, max_rail_update_time_us);
    }
    if (timerDue(current_time, prev_lidar_update_time, lidar_update_delta))
    {
        const uint32_t start = micros();
        sensors.updateLidar();
        recordSensorTiming(micros() - start, lidar_update_count, lidar_update_time_us, max_lidar_update_time_us);
    }
}

void System::reportPerformance()
{
    const uint32_t now_ms = millis();
    if (perf_report_time == 0)
    {
        perf_report_time = now_ms;
        return;
    }

    if (now_ms - perf_report_time < 1000)
    {
        return;
    }

    const uint32_t avg_fast = fast_path_count ? fast_path_time_us / fast_path_count : 0;
    const uint32_t avg_sensor_fast = fast_path_count ? sensor_fast_time_us / fast_path_count : 0;
    const uint32_t avg_estimator = fast_path_count ? estimator_time_us / fast_path_count : 0;
    const uint32_t avg_slow_sensor = slow_sensor_count ? slow_sensor_time_us / slow_sensor_count : 0;
    const uint32_t avg_power = power_monitor_count ? power_monitor_time_us / power_monitor_count : 0;
    const uint32_t avg_log = log_path_count ? log_path_time_us / log_path_count : 0;

    const uint32_t avg_gps = gps_update_count ? gps_update_time_us / gps_update_count : 0;
    const uint32_t avg_baro = baro_update_count ? baro_update_time_us / baro_update_count : 0;
    const uint32_t avg_mag = mag_update_count ? mag_update_time_us / mag_update_count : 0;
    const uint32_t avg_rail = rail_update_count ? rail_update_time_us / rail_update_count : 0;
    const uint32_t avg_lidar = lidar_update_count ? lidar_update_time_us / lidar_update_count : 0;

    Serial.printf(
        "PERF hz=%lu fast_avg/max=%lu/%luus imu_avg/max=%lu/%luus ekf_avg/max=%lu/%luus slow_avg/max=%lu/%luus power_avg/max=%lu/%luus log_avg/max=%lu/%luus\n",
        static_cast<unsigned long>(fast_path_count),
        static_cast<unsigned long>(avg_fast),
        static_cast<unsigned long>(max_fast_path_time_us),
        static_cast<unsigned long>(avg_sensor_fast),
        static_cast<unsigned long>(max_sensor_fast_time_us),
        static_cast<unsigned long>(avg_estimator),
        static_cast<unsigned long>(max_estimator_time_us),
        static_cast<unsigned long>(avg_slow_sensor),
        static_cast<unsigned long>(max_slow_sensor_time_us),
        static_cast<unsigned long>(avg_power),
        static_cast<unsigned long>(max_power_monitor_time_us),
        static_cast<unsigned long>(avg_log),
        static_cast<unsigned long>(max_log_path_time_us)
    );
    Serial.printf(
        "PERF slow gps=%lu avg/max=%lu/%luus baro=%lu avg/max=%lu/%luus mag=%lu avg/max=%lu/%luus rails=%lu avg/max=%lu/%luus lidar=%lu avg/max=%lu/%luus\n",
        static_cast<unsigned long>(gps_update_count),
        static_cast<unsigned long>(avg_gps),
        static_cast<unsigned long>(max_gps_update_time_us),
        static_cast<unsigned long>(baro_update_count),
        static_cast<unsigned long>(avg_baro),
        static_cast<unsigned long>(max_baro_update_time_us),
        static_cast<unsigned long>(mag_update_count),
        static_cast<unsigned long>(avg_mag),
        static_cast<unsigned long>(max_mag_update_time_us),
        static_cast<unsigned long>(rail_update_count),
        static_cast<unsigned long>(avg_rail),
        static_cast<unsigned long>(max_rail_update_time_us),
        static_cast<unsigned long>(lidar_update_count),
        static_cast<unsigned long>(avg_lidar),
        static_cast<unsigned long>(max_lidar_update_time_us)
    );

    perf_report_time = now_ms;
    fast_path_count = 0;
    slow_sensor_count = 0;
    power_monitor_count = 0;
    log_path_count = 0;
    fast_path_time_us = 0;
    sensor_fast_time_us = 0;
    estimator_time_us = 0;
    slow_sensor_time_us = 0;
    power_monitor_time_us = 0;
    log_path_time_us = 0;
    max_fast_path_time_us = 0;
    max_sensor_fast_time_us = 0;
    max_estimator_time_us = 0;
    max_slow_sensor_time_us = 0;
    max_power_monitor_time_us = 0;
    max_log_path_time_us = 0;

    gps_update_count = 0;
    baro_update_count = 0;
    mag_update_count = 0;
    rail_update_count = 0;
    lidar_update_count = 0;
    gps_update_time_us = 0;
    baro_update_time_us = 0;
    mag_update_time_us = 0;
    rail_update_time_us = 0;
    lidar_update_time_us = 0;
    max_gps_update_time_us = 0;
    max_baro_update_time_us = 0;
    max_mag_update_time_us = 0;
    max_rail_update_time_us = 0;
    max_lidar_update_time_us = 0;
}

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

    // network interfaces
    radio.setup();
    canbus.setup();

    // add interfaces to netmanager
    configureNetwork();
    powermonitor.setup();

    networkmanager.registerService(
        static_cast<uint8_t>(Services::ID::PowerMonitor),
        [this](packetptr_t packetptr) {
            powermonitor.networkCallback(std::move(packetptr));
        }
    );
   
    loadConfig();

    estimator.setup();

    controller.setup();
    networkmanager.registerService(static_cast<uint8_t>(Services::ID::Controller),controller.getThisNetworkCallback());

    // initialize statemachine with preflight state
    statemachine.initalize(std::make_unique<Preflight>(*this));

};

void System::systemUpdate()
{
    const uint32_t current_time = micros();
    static uint32_t ekf_debug_last_us = 0;
    static uint32_t ekf_debug_count = 0;
    static uint32_t ekf_debug_total_time_us = 0;
    static uint32_t ekf_debug_max_time_us = 0;

    if (prev_estimator_update_time == 0)
    {
        prev_estimator_update_time = current_time;
    }

    if (current_time - prev_estimator_update_time >= estimator_update_delta)
    {
        const uint32_t fast_start = micros();
        sensors.updateFast();
        const uint32_t estimator_start = micros();
        estimator.update(sensors.getData());
        const uint32_t fast_end = micros();

        const uint32_t sensor_dt = estimator_start - fast_start;
        const uint32_t estimator_dt = fast_end - estimator_start;
        const uint32_t fast_dt = fast_end - fast_start;

        sensor_fast_time_us += sensor_dt;
        estimator_time_us += estimator_dt;
        fast_path_time_us += fast_dt;
        fast_path_count++;
        if (sensor_dt > max_sensor_fast_time_us)
        {
            max_sensor_fast_time_us = sensor_dt;
        }
        if (estimator_dt > max_estimator_time_us)
        {
            max_estimator_time_us = estimator_dt;
        }
        if (fast_dt > max_fast_path_time_us)
        {
            max_fast_path_time_us = fast_dt;
        }

        prev_estimator_update_time += estimator_update_delta;
        if (current_time - prev_estimator_update_time >= estimator_update_delta)
        {
            prev_estimator_update_time = current_time;
        }
    }

    const uint32_t slow_start = micros();
    updateSlowSensors(current_time);
    const uint32_t slow_dt = micros() - slow_start;
    slow_sensor_time_us += slow_dt;
    slow_sensor_count++;
    if (slow_dt > max_slow_sensor_time_us)
    {
        max_slow_sensor_time_us = slow_dt;
    }

    // Keep power-monitor CAN requests disabled until CAN is explicitly re-enabled
    // in configureNetwork(); otherwise networkmanager logs invalid-interface errors.
    if (current_time - prev_power_monitor_update_time >= power_monitor_update_delta)
    {
        const uint32_t power_start = micros();
        powermonitor.update();
        const uint32_t power_dt = micros() - power_start;
        power_monitor_time_us += power_dt;
        power_monitor_count++;
        if (power_dt > max_power_monitor_time_us)
        {
            max_power_monitor_time_us = power_dt;
        }

        prev_power_monitor_update_time += power_monitor_update_delta;
        if (current_time - prev_power_monitor_update_time >= power_monitor_update_delta)
        {
            prev_power_monitor_update_time = current_time;
        }
    }

    {
        const uint32_t log_start = micros();
        logEstimator();
        const uint32_t log_dt = micros() - log_start;
        log_path_time_us += log_dt;
        log_path_count++;
        if (log_dt > max_log_path_time_us)
        {
            max_log_path_time_us = log_dt;
        }
    }
    logTelemetry();

    //reportPerformance();
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
    //! Pulling up for now Will change when we write
    //! the active current monitor
    #if HARDWARE_VERSION == 3
        pinMode(PinMap::DepSwitch, OUTPUT);
        digitalWrite(PinMap::DepSwitch, HIGH); 
    #endif
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
        // estimator.configure(configDoc.as<JsonObjectConst>()["Estimator"]);

        sensors.setup(configDoc.as<JsonObjectConst>()["Sensors"]);

    }
    catch (const std::exception &e)
    {
         RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Exception occured while loading flight config! - " + std::string(e.what()));

         throw e; //continue throwing as we dont want to continue
    }
   
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
    std::unique_ptr<WrappedFile> estimatorlogfile = primarysd.open(log_directory_path + "/estimatorlog.txt",static_cast<FILE_MODE>(O_WRITE | O_CREAT | O_AT_END),10);
    std::unique_ptr<WrappedFile> controllerlogfile = primarysd.open(log_directory_path + "/controller.txt",static_cast<FILE_MODE>(O_WRITE | O_CREAT | O_AT_END),10);
    
    // intialize sys logger
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::SYS>().initialize(std::move(syslogfile),networkmanager);
   
    //initialize telemetry logger
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::TELEMETRY>().initialize(std::move(telemetrylogfile));

    //initialize estimator logger
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::ESTIMATOR>().initialize(std::move(estimatorlogfile));

    //initialize controller logger
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::CONTROLLER>().initialize(std::move(controllerlogfile));

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("SD Init Complete");
}

void System::logEstimator()
{
    const uint32_t current_time = micros();

    if (prev_estimator_log_time == 0)
    {
        prev_estimator_log_time = current_time;
    }

    if (current_time - prev_estimator_log_time >= estimator_log_delta)
    {
        const SensorStructs::state_t& state = estimator.getData();
        EstimatorLogframe logframe;

        logframe.timestamp_us          = esp_timer_get_time();

        logframe.raw_ax                = state.rawAccel(0);
        logframe.raw_ay                = state.rawAccel(1);
        logframe.raw_az                = state.rawAccel(2);
        logframe.raw_gx                = state.rawGyro(0);
        logframe.raw_gy                = state.rawGyro(1);
        logframe.raw_gz                = state.rawGyro(2);
        logframe.filtered_ax           = state.filteredAccel(0);
        logframe.filtered_ay           = state.filteredAccel(1);
        logframe.filtered_az           = state.filteredAccel(2);
        logframe.filtered_gx           = state.filteredGyro(0);
        logframe.filtered_gy           = state.filteredGyro(1);
        logframe.filtered_gz           = state.filteredGyro(2);
        logframe.controller_batt_V     = controller.getBatteryVoltage();
        logframe.controller_voltage_scale = controller.getVoltageScale();
        logframe.controller_thrust_top_cmd = controller.getCommandedThrustTop();
        logframe.controller_thrust_bottom_cmd = controller.getCommandedThrustBottom();
        logframe.controller_fx_cmd = controller.getFxCmd();
        logframe.controller_fx_cmd_outer = controller.getFxCmdOuter();
        logframe.controller_position_control_enabled = controller.getPositionControlEnabled() ? 1 : 0;
        const Eigen::Vector3f controller_pos_err = controller.getPositionError();
        const Eigen::Vector3f controller_vel_err = controller.getVelocityError();
        logframe.controller_pos_err_x = controller_pos_err(0);
        logframe.controller_pos_err_y = controller_pos_err(1);
        logframe.controller_pos_err_z = controller_pos_err(2);
        logframe.controller_vel_err_x = controller_vel_err(0);
        logframe.controller_vel_err_y = controller_vel_err(1);
        logframe.controller_vel_err_z = controller_vel_err(2);

        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::ESTIMATOR>(logframe);

        prev_estimator_log_time += estimator_log_delta;
        if (current_time - prev_estimator_log_time >= estimator_log_delta)
        {
            prev_estimator_log_time = current_time;
        }
    }
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
        const PowerMonitorData& power_data = powermonitor.getData();
        TelemetryLogframe logframe;
        
        logframe.gps_lat = raw_sensors.gps.latitude * 1e-7;
        logframe.gps_long = raw_sensors.gps.longitude * 1e-7;
        logframe.gps_alt = raw_sensors.gps.altitude;
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
        logframe.baro_temp = raw_sensors.baro.temp;
        logframe.baro_press = raw_sensors.baro.press;
        logframe.logic_voltage = raw_sensors.logicrail.volt;
        logframe.logic_percent = raw_sensors.logicrail.percent;
        logframe.dep_voltage = raw_sensors.deprail.volt;
        logframe.dep_current = raw_sensors.deprail.current;
    
        logframe.pdb_batt_mV = power_data.batt_mV;
        logframe.pdb_batt_fresh = power_data.fresh ? 1 : 0;

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

        logframe.timestamp = esp_timer_get_time();

        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::TELEMETRY>(logframe);

        prev_telemetry_log_time = esp_timer_get_time();
    }
}

void System::configureNetwork()
{   
    networkmanager.setNodeType(NODETYPE::HUB);
    networkmanager.addInterface(&radio);
    networkmanager.addInterface(&canbus);

    networkmanager.enableAutoRouteGen(true);
    networkmanager.setNoRouteAction(NOROUTE_ACTION::DUMP, {1,2});

    RoutingTable flightRouting;

    #if ROCKET_TABLE
        flightRouting.setRoute((uint8_t)   5, Route{2, 1, {}}); // Rocket GS Pickle
        flightRouting.setRoute((uint8_t) 20, Route{3, 1, {}}); // PDU0 / LightningMcQueen
        flightRouting.setRoute((uint8_t) 102, Route{3, 2, {}}); // chad srvo
        flightRouting.setRoute((uint8_t) 103, Route{3, 2, {}}); // chad prop
    #elif ROCKET_GS_TABLE
        flightRouting.setRoute((uint8_t)   2, Route{2, 1, {}}); // Rocket Pickle
        flightRouting.setRoute((uint8_t) 20, Route{3, 1, {}}); // PDU0 / LightningMcQueen
        flightRouting.setRoute((uint8_t) 102, Route{2, 2, {}}); // chad srvo
        flightRouting.setRoute((uint8_t) 103, Route{2, 2, {}}); // chad prop
    #endif
    
    networkmanager.setRoutingTable(flightRouting);
    networkmanager.updateBaseTable(); // save the new base table
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(flightRouting.printTable().str());
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
