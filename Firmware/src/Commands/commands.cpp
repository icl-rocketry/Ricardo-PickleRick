/**
 * @file commands.cpp
 * @author Kiran de Silva (kd619@ic.ac.uk)
 * @brief Implementation of commands for system
 * @version 0.1
 * @date 2023-06-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "Commands/commands.h"
#include "Commands/packets/updatemagcalpacket.h"
#include "Commands/packets/rawmagpacket.h"
#include "Commands/packets/radiotestpacket.h"
#include "Commands/packets/telemetrypacket.h"
#include "Commands/packets/sensorspacket.h"
#include "Commands/packets/estimatorpacket.h"
#include "Config/services_config.h"
#include "States/debug.h"
#include "States/flight.h"
#include "States/launch.h"
#include "States/preflight.h"
#include "States/recovery.h"
#include "system.h"

void Commands::LaunchCommand(System& system, const RnpPacketSerialized& packet)
{
    system.statemachine.changeState(std::make_unique<Launch>(system));
}

void Commands::ResetCommand(System& system, const RnpPacketSerialized& packet)
{
    system.statemachine.changeState(std::make_unique<Preflight>(system));
}

void Commands::LaunchAbortCommand(System& system, const RnpPacketSerialized& packet)
{
    // if(system.systemstatus.flagSetOr(SYSTEM_FLAG::STATE_LAUNCH)){
    // 	//check if we are in no abort time region
    // 	//close all valves
    // 	system.statemachine.changeState(new Preflight(&system));
    // }else if (system.systemstatus.flagSetOr(SYSTEM_FLAG::STATE_FLIGHT)){
    // 	//this behaviour needs to be confirmed with recovery
    // 	//might be worth waiting for acceleration to be 0 after rocket engine cut
    // 	system.statemachine.changeState(new Recovery(&system));
    // }

    // TODO log
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(
        "Launch Aborted, Entering Preflight state");
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Disarming all engines and deployers");
    system.enginehandler.disarmComponents();
    system.deploymenthandler.disarmComponents();

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Resetting event handler");
    system.eventhandler.reset();
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Reset Ignition Time");
    // system.estimator.setIgnitionTime(0);

    system.statemachine.changeState(std::make_unique<Preflight>(system));
}

void Commands::FlightAbortCommand(System& system, const RnpPacketSerialized& packet)
{
    // flight abort
    // TODO log
    system.statemachine.changeState(std::make_unique<Recovery>(system));
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(
        "Flight Aborted, Entering recovery state");
}

void Commands::SetHomeCommand(System& system, const RnpPacketSerialized& packet)
{
    // if(!system.systemstatus.flagSetOr(SYSTEM_FLAG::DEBUG)){
    // 	return;
    // }
    // system.estimator.setHome(system.sensors.getData());
    system.tunezhandler.play(MelodyLibrary::confirmation);  // play sound when complete
}

void Commands::StartLoggingCommand(System& system, const RnpPacketSerialized& packet)
{
    SimpleCommandPacket commandpacket(packet);
    // system.logcontroller.startLogging((LOG_TYPE)commandpacket.arg);
}

void Commands::StopLoggingCommand(System& system, const RnpPacketSerialized& packet)
{
    SimpleCommandPacket commandpacket(packet);
    // system.logcontroller.stopLogging((LOG_TYPE)commandpacket.arg);
}


void Commands::TelemetryCommand(System& system, const RnpPacketSerialized& packet)
{
    SimpleCommandPacket commandpacket(packet);

    TelemetryPacket telemetry;

    auto raw_sensors = system.sensors.getData();

    telemetry.header.type = 101;
    telemetry.header.source = system.networkmanager.getAddress();

    telemetry.header.source_service = static_cast<uint8_t>(DEFAULT_SERVICES::COMMAND);
    telemetry.header.destination = commandpacket.header.source;
    telemetry.header.destination_service = commandpacket.header.source_service;
    telemetry.header.uid = commandpacket.header.uid;

    
    telemetry.ax        = raw_sensors.accelgyro.ax;
    telemetry.ay        = raw_sensors.accelgyro.ay;
    telemetry.az        = raw_sensors.accelgyro.az;
    telemetry.gx        = raw_sensors.accelgyro.gx;
    telemetry.gy        = raw_sensors.accelgyro.gy;
    telemetry.gz        = raw_sensors.accelgyro.gz;
    
    telemetry.h_ax      = raw_sensors.accel.ax;
    telemetry.h_ay      = raw_sensors.accel.ay;
    telemetry.h_az      = raw_sensors.accel.az;
    
    telemetry.mx        = raw_sensors.mag.mx;
    telemetry.my        = raw_sensors.mag.my;
    telemetry.mz        = raw_sensors.mag.mz;
    
    telemetry.baro_temp = raw_sensors.baro.temp;
    telemetry.baro_press = raw_sensors.baro.press;
    
    telemetry.latitude  = raw_sensors.gps.latitude;
    telemetry.longitude = raw_sensors.gps.longitude;
    telemetry.altitude  = raw_sensors.gps.altitude;
    telemetry.sat       = raw_sensors.gps.sat;
    
    telemetry.system_status = system.systemstatus.getStatus();
    telemetry.system_time = millis();


    system.networkmanager.sendPacket(telemetry);
}

void Commands::SensorsCommand(System& system, const RnpPacketSerialized& packet)
{
    SimpleCommandPacket commandpacket(packet);

    SensorsPacket sensors;

    auto raw_sensors = system.sensors.getData();

    sensors.header.type = 101;
    sensors.header.source = system.networkmanager.getAddress();

    sensors.header.source_service = static_cast<uint8_t>(DEFAULT_SERVICES::COMMAND);
    sensors.header.destination = commandpacket.header.source;
    sensors.header.destination_service = commandpacket.header.source_service;
    sensors.header.uid = commandpacket.header.uid;

    
    sensors.ax        = raw_sensors.accelgyro.ax;
    sensors.ay        = raw_sensors.accelgyro.ay;
    sensors.az        = raw_sensors.accelgyro.az;
    sensors.gx        = raw_sensors.accelgyro.gx;
    sensors.gy        = raw_sensors.accelgyro.gy;
    sensors.gz        = raw_sensors.accelgyro.gz;
    sensors.temp_ag   = raw_sensors.accelgyro.temp;
    
    sensors.h_ax      = raw_sensors.accel.ax;
    sensors.h_ay      = raw_sensors.accel.ay;
    sensors.h_az      = raw_sensors.accel.az;
    
    sensors.mx        = raw_sensors.mag.mx;
    sensors.my        = raw_sensors.mag.my;
    sensors.mz        = raw_sensors.mag.mz;
    sensors.temp_m    = raw_sensors.mag.temp;
    
    sensors.baro_temp = raw_sensors.baro.temp;
    sensors.baro_press = raw_sensors.baro.press;
    
    sensors.latitude  = raw_sensors.gps.latitude;
    sensors.longitude = raw_sensors.gps.longitude;
    sensors.altitude  = raw_sensors.gps.altitude;
    sensors.v_n       = raw_sensors.gps.v_n;
    sensors.v_e       = raw_sensors.gps.v_e;
    sensors.v_d       = raw_sensors.gps.v_d;
    sensors.hAcc      = raw_sensors.gps.hAcc;
    sensors.vAcc      = raw_sensors.gps.vAcc;
    sensors.sat       = raw_sensors.gps.sat;
    sensors.fix       = raw_sensors.gps.fix;
    
    sensors.system_status = system.systemstatus.getStatus();
    sensors.system_time = millis();


    system.networkmanager.sendPacket(sensors);
}

void Commands::EstimatorCommand(System& system, const RnpPacketSerialized& packet)
{
    SimpleCommandPacket commandpacket(packet);

    EstimatorPacket estimator;

    auto state = system.estimator.getData();

    estimator.header.source =               system.networkmanager.getAddress();
    estimator.header.source_service =       static_cast<uint8_t>(DEFAULT_SERVICES::COMMAND);
    estimator.header.destination =          commandpacket.header.source;
    estimator.header.destination_service =  commandpacket.header.source_service;
    estimator.header.uid =                  commandpacket.header.uid;

    estimator.pos_n                 = state.position(0);
    estimator.pos_e                 = state.position(1);
    estimator.pos_d                 = state.position(2);

    estimator.vel_n                 = state.velocity(0);
    estimator.vel_e                 = state.velocity(1);
    estimator.vel_d                 = state.velocity(2);

    estimator.acc_n                 = state.acceleration(0);
    estimator.acc_e                 = state.acceleration(1);
    estimator.acc_d                 = state.acceleration(2);

    estimator.q0                    = state.orientation.w();
    estimator.q1                    = state.orientation.x();
    estimator.q2                    = state.orientation.y();
    estimator.q3                    = state.orientation.z();

    estimator.b_gx                  = state.gyroBiases(0);
    estimator.b_gy                  = state.gyroBiases(1);
    estimator.b_gz                  = state.gyroBiases(2);

    estimator.b_ax                  = state.accelBiases(0);
    estimator.b_ay                  = state.accelBiases(1);
    estimator.b_az                  = state.accelBiases(2);

    estimator.calibration_quality   = state.calibration_quality;

    estimator.h_mx                  = state.expectedMagReading(0);
    estimator.h_my                  = state.expectedMagReading(1);
    estimator.h_mz                  = state.expectedMagReading(2);

    estimator.h_ax                  = state.expectedAccelReading(0);
    estimator.h_ay                  = state.expectedAccelReading(1);
    estimator.h_az                  = state.expectedAccelReading(2);

    estimator.y_mx                  = state.magInnovation(0);
    estimator.y_my                  = state.magInnovation(1);
    estimator.y_mz                  = state.magInnovation(2);

    estimator.y_ax                  = state.accelInnovation(0);
    estimator.y_ay                  = state.accelInnovation(1);
    estimator.y_az                  = state.accelInnovation(2);

    estimator.b_hax                 = state.highGBiases(0);
    estimator.b_hay                 = state.highGBiases(1);
    estimator.b_haz                 = state.highGBiases(2);

    estimator.ref_mn                = state.refMag(0);
    estimator.ref_me                = state.refMag(1);
    estimator.ref_md                = state.refMag(2);

    estimator.system_status         = system.systemstatus.getStatus();
    estimator.system_time           = millis();

    system.networkmanager.sendPacket(estimator);
}

void Commands::CalibrateEstimatorCommand(System& system, const RnpPacketSerialized& packet)
{

    system.estimator.calibrate();

}

void Commands::PlaySongCommand(System& system, const RnpPacketSerialized& packet)
{
    SimpleCommandPacket commandpacket(packet);
    system.tunezhandler.play_by_idx(commandpacket.arg);
}

void Commands::SkipSongCommand(System& system, const RnpPacketSerialized& packet)
{
    system.tunezhandler.skip();
}

void Commands::ClearSongQueueCommand(System& system, const RnpPacketSerialized& packet)
{
    system.tunezhandler.clear();
}

void Commands::ResetOrientationCommand(System& system, const RnpPacketSerialized& packet)
{
    // system.estimator.resetOrientation();
    system.tunezhandler.play(MelodyLibrary::confirmation);  // play sound when complete
}

void Commands::ResetLocalizationCommand(System& system, const RnpPacketSerialized& packet)
{
    // system.estimator.resetLocalization();
    system.tunezhandler.play(MelodyLibrary::confirmation);  // play sound when complete
}

void Commands::SetBetaCommand(System& system, const RnpPacketSerialized& packet)
{
    SimpleCommandPacket commandpacket(packet);
    float beta = ((float)commandpacket.arg) / 100.0;
    // system.estimator.changeBeta(beta);
}

void Commands::MagTelemetryCommand(System& system, const RnpPacketSerialized& packet)
{
    SimpleCommandPacket commandpacket(packet);

    RawMagPacket telemetry;

    auto raw_data = system.sensors.getRawMagData();

    telemetry.header.type = 101;
    telemetry.header.source = system.networkmanager.getAddress();

    telemetry.header.source_service = static_cast<uint8_t>(DEFAULT_SERVICES::COMMAND);
    telemetry.header.destination = commandpacket.header.source;
    telemetry.header.destination_service = commandpacket.header.source_service;
    telemetry.header.uid = commandpacket.header.uid;

    
    telemetry.mx        = raw_data(0);
    telemetry.my        = raw_data(1);
    telemetry.mz        = raw_data(2);

    system.networkmanager.sendPacket(telemetry);
}

void Commands::CalibrateMagFullCommand(System& system, const RnpPacketSerialized& packet)
{
    // check mag cal (id 10) packet type received
    if (packet.header.type != 10)
    {
        // incorrect packet type received do not deserialize
        // TODO log
        return;
    }

    UpdateMagCalPacket magcalpacket(packet);
    system.sensors.calibrateMag(MagCalibrationParameters{magcalpacket.getA(), magcalpacket.getB()});
    system.tunezhandler.play(MelodyLibrary::confirmation);  // play sound when complete
}

void Commands::IgnitionCommand(System& system, const RnpPacketSerialized& packet)
{
    uint32_t currentTime = millis();
    // system.estimator.setIgnitionTime(currentTime);  // set igintion time
}

void Commands::EnterDebugCommand(System& system, const RnpPacketSerialized& packet)
{
    system.statemachine.changeState(std::make_unique<Debug>(system));
}

void Commands::EnterPreflightCommand(System& system, const RnpPacketSerialized& packet)
{
    system.statemachine.changeState(std::make_unique<Preflight>(system));
}

void Commands::EnterLaunchCommand(System& system, const RnpPacketSerialized& packet)
{
    system.statemachine.changeState(std::make_unique<Launch>(system));
}

void Commands::EnterFlightCommand(System& system, const RnpPacketSerialized& packet)
{
    system.statemachine.changeState(std::make_unique<Flight>(system));
}

void Commands::EnterRecoveryCommand(System& system, const RnpPacketSerialized& packet)
{
    system.statemachine.changeState(std::make_unique<Recovery>(system));
}

void Commands::ExitDebugCommand(System& system, const RnpPacketSerialized& packet)
{
    system.statemachine.changeState(std::make_unique<Debug>(system));
    system.systemstatus.deleteFlag(
        SYSTEM_FLAG::DEBUG);  // delete system flag to signify exiting debug mode
    system.statemachine.changeState(std::make_unique<Preflight>(system));
}

void Commands::LiftoffOverrideCommand(System& system, const RnpPacketSerialized& packet)
{
    // system.estimator.setLiftoffTime(millis());
    system.tunezhandler.play(MelodyLibrary::confirmation);  // play sound when complete
    system.statemachine.changeState(std::make_unique<Flight>(system));
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(
        "Liftoff Override triggered, Forcing into flight mode!");
}

void Commands::FreeRamCommand(System& system, const RnpPacketSerialized& packet)
{

    SimpleCommandPacket commandpacket(packet);

    uint32_t freeram = esp_get_free_heap_size();
    // avliable in all states
    // returning as simple string packet for ease
    // currently only returning free ram
    if (commandpacket.arg == 0)
    {
        MessagePacket_Base<0, static_cast<uint8_t>(
                                  decltype(System::commandhandler)::PACKET_TYPES::MESSAGE_RESPONSE)>
            message("FreeRam: " + std::to_string(esp_get_free_heap_size()));
        // this is not great as it assumes a single command handler with the same service ID
        // would be better if we could pass some context through the function paramters so it has an
        // idea who has called it or make it much clearer that only a single command handler should
        // exist in the system
        message.header.source_service = system.commandhandler.getServiceID();
        message.header.destination_service = packet.header.source_service;
        message.header.source = packet.header.destination;
        message.header.destination = packet.header.source;
        message.header.uid = packet.header.uid;
        system.networkmanager.sendPacket(message);
    }
    else if (commandpacket.arg == 1)
    {
        BasicDataPacket<uint32_t, 0, 105> responsePacket(freeram);
        responsePacket.header.source_service = system.commandhandler.getServiceID();
        responsePacket.header.destination_service = packet.header.source_service;
        responsePacket.header.source = packet.header.destination;
        responsePacket.header.destination = packet.header.source;
        responsePacket.header.uid = packet.header.uid;
        system.networkmanager.sendPacket(responsePacket);
    }
}

void Commands::ApogeeOverrideCommand(System& system, const RnpPacketSerialized& packet)
{
    SimpleCommandPacket commandpacket(packet);

    system.systemstatus.newFlag(SYSTEM_FLAG::FLIGHTPHASE_APOGEE, "Apogee Triggered!");
    // system.estimator.setApogeeTime(millis());

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(
        "Apogee Time Overriden, transitioning to Recovery State!");
    system.statemachine.changeState(std::make_unique<Recovery>(system));
}

//! TEMP
void Commands::Radio_SetFreq(System& system, const RnpPacketSerialized& packet)
{
    SimpleCommandPacket commandpacket(packet);

    long frequency = commandpacket.arg;

    system.radio.setFreq(frequency);
}

void Commands::Radio_SetBW(System& system, const RnpPacketSerialized& packet)
{
    SimpleCommandPacket commandpacket(packet);

    long bandwidth = commandpacket.arg;

    system.radio.setBW(bandwidth);
}

void Commands::Radio_SetSF(System& system, const RnpPacketSerialized& packet)
{
    SimpleCommandPacket commandpacket(packet);

    uint8_t SF = static_cast<uint8_t>(commandpacket.arg);

    system.radio.setSF(SF);
}

void Commands::Radio_SetPower(System& system, const RnpPacketSerialized& packet)
{
    SimpleCommandPacket commandpacket(packet);

    uint8_t Power = static_cast<uint8_t>(commandpacket.arg);

    system.radio.setPower(Power);
}

void Commands::Radio_SetSYNC(System& system, const RnpPacketSerialized& packet)
{
    SimpleCommandPacket commandpacket(packet);

    uint8_t SW = static_cast<uint8_t>(commandpacket.arg);

    system.radio.setSW(SW);
}