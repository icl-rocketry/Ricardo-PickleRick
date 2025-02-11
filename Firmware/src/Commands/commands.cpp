#include "commands.h"

#include <memory>

#include <librnp/rnp_packet.h>
#include <librnp/rnp_interface.h>
#include <librnp/rnp_networkmanager.h>
#include <libriccore/commands/commandhandler.h>

#include "packets/magcalcommandpacket.h"
#include "packets/TelemetryPacket.h"

#include "system.h"

#include "Config/services_config.h"






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
	// this is not great as it assumes a single command handler with the same service ID
	// would be better if we could pass some context through the function paramters so it has an idea who has called it
	// or make it much clearer that only a single command handler should exist in the system
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

	telemetry.q0 = estimator_state.orientation.w();
	telemetry.q1 = estimator_state.orientation.x();
	telemetry.q2 =estimator_state.orientation.y();
	telemetry.q3 =estimator_state.orientation.z();

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

	telemetry.batt_voltage = raw_sensors.logicrail.volt;
	telemetry.batt_percent= raw_sensors.logicrail.percent;

	telemetry.launch_lat = estimator_state.gps_launch_lat;
	telemetry.launch_lng = estimator_state.gps_launch_long;
	telemetry.launch_alt = estimator_state.gps_launch_alt;

	telemetry.system_status = system.systemstatus.getStatus();
	

	const RadioInterfaceInfo* radioinfo = static_cast<const RadioInterfaceInfo*>(system.radio.getInfo());
	telemetry.rssi = radioinfo->rssi;
	telemetry.snr = radioinfo->snr;



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
	system.estimator.setup();
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