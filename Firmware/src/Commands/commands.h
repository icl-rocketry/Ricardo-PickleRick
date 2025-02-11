
#pragma once
#include <stdint.h>

// Forward declaration required here as commands.h contains command function signatures, so is included in Config/commands_config.h which is 
// then included in system.h as the command id enum type is required in the generation of the riccoresystem template. Better solutions may
//exist but currently this seems okay
#include "Config/forward_decl.h" 

#include <librnp/rnp_packet.h>

namespace Commands{
    
    void SetHomeCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void StartLoggingCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void StopLoggingCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void TelemetryCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void PlaySongCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void SkipSongCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void ClearSongQueueCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void ResetOrientationCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void ResetLocalizationCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void SetBetaCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void CalibrateAccelGyroBiasCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void CalibrateHighGAccelBiasCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void CalibrateMagFullCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void CalibrateBaroCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
    void FreeRamCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);
}