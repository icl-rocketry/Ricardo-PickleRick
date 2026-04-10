#pragma once
#include <stdint.h>

// Forward declaration required here as commands.h contains command function signatures, so is included in Config/commands_config.h which is 
// then included in system.h as the command id enum type is required in the generation of the riccoresystem template. Better solutions may
//exist but currently this seems okay
#include "Config/forward_decl.h" 

#include <libriccore/commands/commandhandler.h>
#include <librnp/rnp_interface.h>
#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>
#include <memory>

namespace Commands{
    
    void SetHomeCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void StartLoggingCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void StopLoggingCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void TelemetryCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void SensorsCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void EstimatorCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void CalibrateEstimatorCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void MagTelemetryCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void CalibrateMagFullCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void EnterPreflightCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void EnterFlightCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void FreeRamCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);

}