#pragma once

#include <tuple>

#include <libriccore/logging/loggers/coutlogger.h>
#include <libriccore/logging/loggers/rnpmessagelogger.h>
#include <libriccore/logging/loggers/syslogger.h>
#include "Loggers/TelemetryLogger/telemetrylogger.h"
#include "Loggers/ApogeeLogger/apogeelogger.h"


namespace RicCoreLoggingConfig
{
    enum class LOGGERS
    {
        SYS, // default system logging
        TELEMETRY,
        APOGEE,
        COUT // cout logging
    };

    extern std::tuple<SysLogger,TelemetryLogger,ApogeeLogger,CoutLogger> logger_list;
}; 


