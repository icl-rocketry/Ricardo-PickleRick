#pragma once

#include <stdint.h>
#include <unordered_map>
#include <functional>
#include <initializer_list>

#include <libriccore/commands/commandhandler.h>
#include <librnp/rnp_packet.h>

#include "Config/forward_decl.h"
#include "Commands/commands.h"


namespace Commands
{
    enum class ID : uint8_t
    {
        Nocommand = 0,
        Telemetry = 8,
        Reset_Orientation = 50,
        Reset_Localization = 51,
        Set_Beta = 52,
        Calibrate_AccelGyro_Bias = 60, // bias callibration requires sensor z axis aligned with up directioN!
        Calibrate_Mag_Full = 61, //changed for compatibility
        Calibrate_HighGAccel_Bias = 62,
        Calibrate_Baro = 63,
        Enter_Debug = 100,
        Exit_Debug = 106,
        Apogee_Override = 131,
        Free_Ram = 250
    };

    inline std::initializer_list<ID> defaultEnabledCommands = {ID::Free_Ram,ID::Telemetry};

    inline std::unordered_map<ID, std::function<void(ForwardDecl_SystemClass &, const RnpPacketSerialized &)>> command_map{
        {ID::Telemetry, TelemetryCommand},
        {ID::Calibrate_AccelGyro_Bias, CalibrateAccelGyroBiasCommand},
        {ID::Calibrate_HighGAccel_Bias, CalibrateHighGAccelBiasCommand},
        {ID::Calibrate_Mag_Full, CalibrateMagFullCommand},
        {ID::Calibrate_Baro, CalibrateBaroCommand},
        {ID::Set_Beta, SetBetaCommand},
        {ID::Reset_Orientation, ResetOrientationCommand},
        {ID::Reset_Localization, ResetLocalizationCommand},
        {ID::Enter_Debug, EnterDebugCommand},
        {ID::Exit_Debug, ExitDebugCommand},
        {ID::Free_Ram, FreeRamCommand},
        {ID::Apogee_Override, ApogeeOverrideCommand}
    };
};