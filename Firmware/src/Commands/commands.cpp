#include "Commands/commands.h"
#include "Commands/packets/updatemagcalpacket.h"
#include "Commands/packets/rawmagpacket.h"
#include "Commands/packets/telemetrypacket.h"
#include "Commands/packets/sensorspacket.h"
#include "Commands/packets/estimatorpacket.h"
#include "Config/services_config.h"
#include "States/flight.h"
#include "States/preflight.h"
#include "States/landing.h"
#include "system.h"

void Commands::SetHomeCommand(System& system, const RnpPacketSerialized& packet)
{
    system.estimator.setHome();
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
    auto estimation = system.estimator.getData();
    telemetry.header.type = 101;
    telemetry.header.source = system.networkmanager.getAddress();

    telemetry.header.source_service = static_cast<uint8_t>(DEFAULT_SERVICES::COMMAND);
    telemetry.header.destination = commandpacket.header.source;
    telemetry.header.destination_service = commandpacket.header.source_service;
    telemetry.header.uid = commandpacket.header.uid;


    
    telemetry.pn        = estimation.position(0);
    telemetry.pe        = estimation.position(1);
    telemetry.pd        = estimation.position(2);

    telemetry.vn        = estimation.velocity(0);
    telemetry.ve        = estimation.velocity(1);
    telemetry.vd        = estimation.velocity(2);

    telemetry.q0        = estimation.orientation.w();
    telemetry.q1        = estimation.orientation.x();
    telemetry.q2        = estimation.orientation.y();
    telemetry.q3        = estimation.orientation.z();

    telemetry.roll      = estimation.eulerAngles(0) * 180.0 / M_PI;
    telemetry.pitch     = estimation.eulerAngles(1) * 180.0 / M_PI;
    telemetry.yaw       = estimation.eulerAngles(2) * 180.0 / M_PI;

    telemetry.rocket_q0        = estimation.rocketOrientation.w();
    telemetry.rocket_q1        = estimation.rocketOrientation.x();
    telemetry.rocket_q2        = estimation.rocketOrientation.y();
    telemetry.rocket_q3        = estimation.rocketOrientation.z();

    telemetry.rocket_roll      = estimation.rocketEulerAngles(0) * 180.0 / M_PI;
    telemetry.rocket_pitch     = estimation.rocketEulerAngles(1) * 180.0 / M_PI;
    telemetry.rocket_yaw       = estimation.rocketEulerAngles(2) * 180.0 / M_PI;

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
    
    estimator.q0                    = state.orientation.w();
    estimator.q1                    = state.orientation.x();
    estimator.q2                    = state.orientation.y();
    estimator.q3                    = state.orientation.z();

    estimator.gps_pos_n             = state.gpsPosition(0);
    estimator.gps_pos_e             = state.gpsPosition(1);
    estimator.gps_pos_d             = state.gpsPosition(2);
    
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

    estimator.h_bt                  = state.expectedBaroReading(0);
    estimator.h_bp                  = state.expectedBaroReading(1);

    estimator.h_pn                  = state.expectedGpsPosReading(0);
    estimator.h_pe                  = state.expectedGpsPosReading(1);
    estimator.h_pd                  = state.expectedGpsPosReading(2);

    estimator.h_vn                  = state.expectedGpsVelReading(0);
    estimator.h_ve                  = state.expectedGpsVelReading(1);
    estimator.h_vd                  = state.expectedGpsVelReading(2);

    estimator.y_mx                  = state.magInnovation(0);
    estimator.y_my                  = state.magInnovation(1);
    estimator.y_mz                  = state.magInnovation(2);

    estimator.y_ax                  = state.accelInnovation(0);
    estimator.y_ay                  = state.accelInnovation(1);
    estimator.y_az                  = state.accelInnovation(2);

    estimator.y_bt                  = state.baroInnovation(0);
    estimator.y_bp                  = state.baroInnovation(1);

    estimator.y_pn                  = state.gpsPosInnovation(0);
    estimator.y_pe                  = state.gpsPosInnovation(1);
    estimator.y_pd                  = state.gpsPosInnovation(2);

    estimator.y_vn                  = state.gpsVelInnovation(0);
    estimator.y_ve                  = state.gpsVelInnovation(1);
    estimator.y_vd                  = state.gpsVelInnovation(2);

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
}

void Commands::EnterPreflightCommand(System& system, const RnpPacketSerialized& packet)
{
    system.statemachine.changeState(std::make_unique<Preflight>(system));
}

void Commands::EnterFlightCommand(System& system, const RnpPacketSerialized& packet)
{
    system.statemachine.changeState(std::make_unique<Flight>(system));
}

void Commands::EnterLandingCommand(System& system, const RnpPacketSerialized& packet)
{
    system.statemachine.changeState(std::make_unique<Landing>(system));
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
