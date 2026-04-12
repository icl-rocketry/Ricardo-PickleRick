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
        Set_Home = 4,
        Start_Logging = 5,
        Stop_Logging = 6,
        Telemetry = 8,
        Sensors = 9,
        Estimator = 10,
        Calibrate = 11,
        Mag_Telemetry = 60, 
        Calibrate_Mag_Full = 61, 
        Enter_Preflight = 100,
        Enter_Flight = 101,
        Enter_Landing = 102,
        Free_Ram = 250
    };

    inline std::initializer_list<ID> defaultEnabledCommands = { ID::Free_Ram,
                                                                ID::Telemetry,
                                                                ID::Sensors,
                                                                ID::Estimator, 
                                                                ID::Enter_Landing, 
                                                            };

    inline std::unordered_map<ID, std::function<void(ForwardDecl_SystemClass &, const RnpPacketSerialized &)>> command_map{
        {ID::Set_Home, SetHomeCommand},
        {ID::Start_Logging, StartLoggingCommand},
        {ID::Stop_Logging, StopLoggingCommand},
        {ID::Telemetry, TelemetryCommand},
        {ID::Sensors, SensorsCommand},
        {ID::Estimator, EstimatorCommand},
        {ID::Calibrate, CalibrateEstimatorCommand},
        {ID::Mag_Telemetry, MagTelemetryCommand},
        {ID::Calibrate_Mag_Full, CalibrateMagFullCommand},
        {ID::Enter_Preflight, EnterPreflightCommand},
        {ID::Enter_Flight, EnterFlightCommand},
        {ID::Enter_Landing, EnterLandingCommand},
        {ID::Free_Ram, FreeRamCommand}};

};