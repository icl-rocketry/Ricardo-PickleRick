#pragma once

#include <tuple>

#include <libriccore/logging/loggers/coutlogger.h>
#include <libriccore/logging/loggers/rnpmessagelogger.h>
#include "Loggers/SysLogger/syslogger.h"
#include "Loggers/TelemetryLogger/telemetrylogger.h"


namespace RicCoreLoggingConfig
{
    enum class LOGGERS
    {
        SYS, // default system logging
        TELEMETRY,
        COUT // cout logging
    };

    extern std::tuple<SysLogger,TelemetryLogger,CoutLogger> logger_list;
}; 


