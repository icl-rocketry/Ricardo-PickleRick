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

#include "commands.h"

#include <memory>

#include <librnp/rnp_packet.h>
#include <librnp/rnp_interface.h>
#include <librnp/rnp_networkmanager.h>
#include <libriccore/commands/commandhandler.h>

#include "packets/magcalcommandpacket.h"
#include "packets/TelemetryPacket.h"
#include "packets/RadioTestPacket.h"

#include "system.h"

#include "States/launch.h"
#include "States/preflight.h"
#include "States/flight.h"
#include "States/recovery.h"
#include "States/debug.h"

#include "Config/services_config.h"


void Commands::LaunchCommand(System& system, const RnpPacketSerialized& packet) 
{
	system.statemachine.changeState(std::make_unique<Launch>(system));
}

void Commands::ResetCommand(System& system, const RnpPacketSerialized& packet) 
{	
	system.statemachine.changeState(std::make_unique<Preflight>(system));
}

void Commands::LaunchAbortCommand(System& system,const  RnpPacketSerialized& packet) 
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
	system.statemachine.changeState(std::make_unique<Preflight>(system));
	//TODO log
	RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Launch Aborted, Entering preflight state");
	RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Disarming all engines and deployers");
	system.enginehandler.disarmComponents();
	system.deploymenthandler.disarmComponents();
}

void Commands::FlightAbortCommand(System& system, const RnpPacketSerialized& packet)
{
	//flight abort
	//TODO log
	system.statemachine.changeState(std::make_unique<Recovery>(system));
	RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Flight Aborted, Entering recovery state");
}

void Commands::SetHomeCommand(System& system, const RnpPacketSerialized& packet) 
{
	// if(!system.systemstatus.flagSetOr(SYSTEM_FLAG::DEBUG)){
	// 	return;
	// }
	system.estimator.setHome(system.sensors.getData());
	system.tunezhandler.play(MelodyLibrary::confirmation); //play sound when complete
	
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
	auto estimator_state = system.estimator.getData();

	telemetry.header.type = 101;
	telemetry.header.source = system.networkmanager.getAddress();
	
	telemetry.header.source_service = static_cast<uint8_t>(DEFAULT_SERVICES::COMMAND);
	telemetry.header.destination = commandpacket.header.source;
	telemetry.header.destination_service = commandpacket.header.source_service;
	telemetry.header.uid = commandpacket.header.uid; 
	telemetry.system_time = millis();

	telemetry.pn = estimator_state.position(0);
	telemetry.pe = estimator_state.position(1);
	telemetry.pd = estimator_state.position(2);

	telemetry.vn = estimator_state.velocity(0);
	telemetry.ve = estimator_state.velocity(1);
	telemetry.vd = estimator_state.velocity(2);

	telemetry.an = estimator_state.acceleration(0);
	telemetry.ae = estimator_state.acceleration(1);
	telemetry.ad = estimator_state.acceleration(2);

	telemetry.roll = estimator_state.eulerAngles(0);
	telemetry.pitch = estimator_state.eulerAngles(1);
	telemetry.yaw =estimator_state.eulerAngles(2);
	// telemetry.yaw = estimator_state.tilt;

	telemetry.q0 = estimator_state.orientation.w();
	telemetry.q1 = estimator_state.orientation.x();
	telemetry.q2 =estimator_state.orientation.y();
	telemetry.q3 =estimator_state.orientation.z();

	// telemetry.roll = estimator_state.rocketEulerAngles(0);
	// telemetry.pitch = estimator_state.rocketEulerAngles(1);
	// telemetry.yaw =estimator_state.rocketEulerAngles(2);
	// // telemetry.yaw = estimator_state.tilt;

	// telemetry.q0 = estimator_state.rocketOrientation.w();
	// telemetry.q1 = estimator_state.rocketOrientation.x();
	// telemetry.q2 =estimator_state.rocketOrientation.y();
	// telemetry.q3 =estimator_state.rocketOrientation.z();

	telemetry.lat = raw_sensors.gps.lat;
	telemetry.lng = raw_sensors.gps.lng;
	telemetry.alt = raw_sensors.gps.alt;
	telemetry.sat = raw_sensors.gps.sat;

	telemetry.ax = raw_sensors.accelgyro.ax;
	telemetry.ay = raw_sensors.accelgyro.ay;
	telemetry.az = raw_sensors.accelgyro.az;

	telemetry.h_ax = raw_sensors.accel.ax;
	telemetry.h_ay = raw_sensors.accel.ay;
	telemetry.h_az = raw_sensors.accel.az;

	telemetry.gx = raw_sensors.accelgyro.gx;
	telemetry.gy = raw_sensors.accelgyro.gy;
	telemetry.gz = raw_sensors.accelgyro.gz;

	telemetry.mx = raw_sensors.mag.mx;
	telemetry.my = raw_sensors.mag.my;
	telemetry.mz = raw_sensors.mag.mz;

	telemetry.baro_temp = raw_sensors.baro.temp;
	telemetry.baro_press = raw_sensors.baro.press;
	telemetry.baro_alt = raw_sensors.baro.alt;

	telemetry.logic_voltage = raw_sensors.logicrail.volt;
	telemetry.dep_voltage = raw_sensors.deprail.volt;
	telemetry.dep_current = raw_sensors.deprail.current;


	telemetry.launch_lat = estimator_state.gps_launch_lat;
	telemetry.launch_lng = estimator_state.gps_launch_long;
	telemetry.launch_alt = estimator_state.gps_launch_alt;

	telemetry.system_status = system.systemstatus.getStatus();
	

	const RadioInterfaceInfo* radioinfo = static_cast<const RadioInterfaceInfo*>(system.radio.getInfo());
	telemetry.rssi = radioinfo->rssi;
	telemetry.snr = radioinfo->snr;



	system.networkmanager.sendPacket(telemetry);

}

//!TEMP
void Commands::RadioTestCommand(System& system, const RnpPacketSerialized& packet) 
{
	SimpleCommandPacket commandpacket(packet);

	RadioTestPacket telemetry;

	telemetry.header.type = 101;
	telemetry.header.source = system.networkmanager.getAddress();
	// this is not great as it assumes a single command handler with the same service ID
	// would be better if we could pass some context through the function paramters so it has an idea who has called it
	// or make it much clearer that only a single command handler should exist in the system
	telemetry.header.source_service = static_cast<uint8_t>(DEFAULT_SERVICES::COMMAND);
	telemetry.header.destination = commandpacket.header.source;
	telemetry.header.destination_service = commandpacket.header.source_service;
	telemetry.header.uid = commandpacket.header.uid; 
	
	telemetry.system_time = millis();
	telemetry.system_status = system.systemstatus.getStatus();
	const RadioInterfaceInfo* radioinfo = static_cast<const RadioInterfaceInfo*>(system.radio.getInfo());
	telemetry.rssi = radioinfo->rssi;
	telemetry.packet_rssi = radioinfo->packet_rssi;
	telemetry.snr = radioinfo->snr;
	telemetry.packet_snr = radioinfo->packet_snr;

	system.networkmanager.sendPacket(telemetry);
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
	
	system.estimator.resetOrientation();
	system.tunezhandler.play(MelodyLibrary::confirmation); //play sound when complete
}

void Commands::ResetLocalizationCommand(System& system, const RnpPacketSerialized& packet) 
{
	system.estimator.resetLocalization();
	system.tunezhandler.play(MelodyLibrary::confirmation); //play sound when complete
}

void Commands::SetBetaCommand(System& system, const RnpPacketSerialized& packet) 
{

	SimpleCommandPacket commandpacket(packet);
	float beta = ((float)commandpacket.arg) / 100.0;
	system.estimator.changeBeta(beta);
}

void Commands::CalibrateAccelGyroBiasCommand(System& system, const RnpPacketSerialized& packet) 
{
	
	system.sensors.calibrateAccelGyro();
	system.tunezhandler.play(MelodyLibrary::confirmation); //play sound when complete
}

void Commands::CalibrateHighGAccelBiasCommand(System& system, const RnpPacketSerialized& packet) 
{
	
	system.sensors.calibrateHighGAccel();
	system.tunezhandler.play(MelodyLibrary::confirmation); //play sound when complete
}

void Commands::CalibrateMagFullCommand(System& system, const RnpPacketSerialized& packet) 
{

	//check mag cal (id 10) packet type received
	if (packet.header.type != 10){
		//incorrect packet type received do not deserialize
		//TODO log
		return;
	}

	MagCalCommandPacket magcalpacket(packet);
	system.sensors.calibrateMag(MagCalibrationParameters{magcalpacket.fieldMagnitude,
													   magcalpacket.inclination,
													   magcalpacket.declination,
													   magcalpacket.getA(),
													   magcalpacket.getB()});
	system.tunezhandler.play(MelodyLibrary::confirmation); // play sound when complete
}

void Commands::CalibrateBaroCommand(System& system, const RnpPacketSerialized& packet)
{
	system.sensors.calibrateBaro();
	system.tunezhandler.play(MelodyLibrary::confirmation); //play sound when complete
}

void Commands::IgnitionCommand(System& system, const RnpPacketSerialized& packet)
{

	uint32_t currentTime = millis();
	system.estimator.setIgnitionTime(currentTime); // set igintion time
	
	
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
	system.systemstatus.deleteFlag(SYSTEM_FLAG::DEBUG); // delete system flag to signify exiting debug mode
	system.statemachine.changeState(std::make_unique<Preflight>(system));
}

void Commands::LiftoffOverrideCommand(System& system, const RnpPacketSerialized& packet) 
{
	system.estimator.setLiftoffTime(millis());
	system.tunezhandler.play(MelodyLibrary::confirmation); //play sound when complete
	system.statemachine.changeState(std::make_unique<Flight>(system));
	RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Liftoff Override triggered, Forcing into flight mode!");
}


void Commands::FreeRamCommand(System& system, const RnpPacketSerialized& packet)
{	
	/// ESP_LOGI("ch", "%s", "deserialize");

	SimpleCommandPacket commandpacket(packet);

	uint32_t freeram = esp_get_free_heap_size();
	//avliable in all states
	//returning as simple string packet for ease
	//currently only returning free ram
	if (commandpacket.arg == 0){
	MessagePacket_Base<0,static_cast<uint8_t>(decltype(System::commandhandler)::PACKET_TYPES::MESSAGE_RESPONSE)> message("FreeRam: " + std::to_string(esp_get_free_heap_size()));
	// this is not great as it assumes a single command handler with the same service ID
	// would be better if we could pass some context through the function paramters so it has an idea who has called it
	// or make it much clearer that only a single command handler should exist in the system
		message.header.source_service = system.commandhandler.getServiceID(); 
		message.header.destination_service = packet.header.source_service;
		message.header.source = packet.header.destination;
		message.header.destination = packet.header.source;
		message.header.uid = packet.header.uid;
		system.networkmanager.sendPacket(message);
	}
	else if (commandpacket.arg == 1)
	{
		BasicDataPacket<uint32_t,0,105> responsePacket(freeram);
		responsePacket.header.source_service = system.commandhandler.getServiceID(); 
		responsePacket.header.destination_service = packet.header.source_service;
		responsePacket.header.source = packet.header.destination;
		responsePacket.header.destination = packet.header.source;
		responsePacket.header.uid = packet.header.uid;
		system.networkmanager.sendPacket(responsePacket);	
	}
	
}

//!TEMP
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