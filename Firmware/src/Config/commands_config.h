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
        Launch = 1, 
        Reset = 2,
        Launch_Abort = 3,
        Set_Home = 4,
        Start_Logging = 5,
        Stop_Logging = 6,
        Telemetry = 8,
        Print_Flash_filesystem = 12,
        Print_Sd_filesystem = 13,
        Play_Song = 14,
        Skip_Song = 15,
        Clear_Song_Queue = 16,
        Refresh_Orientation = 50,
        Reset_Localization = 51,
        Set_Beta = 52,
        Reset_Orientation = 53,
        Calibrate_AccelGyro_Bias = 60, // bias callibration requires sensor z axis aligned with up directioN!
        Calibrate_Mag_Full = 61, //changed for compatibility
        Calibrate_HighGAccel_Bias = 62,
        Calibrate_Baro = 63,
        Free_Ram = 250,
        Enter_Flight = 100,
        Enter_Hard_Abort = 101,
        Enter_Soft_Abort = 102,
        Enter_Land = 103
    };

    inline std::initializer_list<ID> defaultEnabledCommands = {ID::Free_Ram,ID::Telemetry};

    inline std::unordered_map<ID, std::function<void(ForwardDecl_SystemClass &, const RnpPacketSerialized &)>> command_map{
        {ID::Reset, ResetCommand},
        {ID::Set_Home, SetHomeCommand},
        {ID::Start_Logging, StartLoggingCommand},
        {ID::Stop_Logging, StopLoggingCommand},
        {ID::Telemetry, TelemetryCommand},
        {ID::Play_Song, PlaySongCommand},
        {ID::Skip_Song, SkipSongCommand},
        {ID::Clear_Song_Queue, ClearSongQueueCommand},
        {ID::Calibrate_AccelGyro_Bias, CalibrateAccelGyroBiasCommand},
        {ID::Calibrate_HighGAccel_Bias, CalibrateHighGAccelBiasCommand},
        {ID::Calibrate_Mag_Full, CalibrateMagFullCommand},
        {ID::Calibrate_Baro, CalibrateBaroCommand},
        {ID::Set_Beta, SetBetaCommand},
        {ID::Refresh_Orientation, RefreshOrientationCommand},
        {ID::Reset_Orientation, ResetOrientationCommand},
        {ID::Reset_Localization, ResetLocalizationCommand},
        {ID::Free_Ram, FreeRamCommand},
        {ID::Enter_Flight, EnterFlightCommand},
        {ID::Enter_Hard_Abort, EnterHardAbortCommand},
        {ID::Enter_Soft_Abort, EnterSoftAbortCommand},
        {ID::Enter_Land, EnterLandCommand}
        };
};