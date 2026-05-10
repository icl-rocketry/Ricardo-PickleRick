#pragma once

#include <tuple>

#include <libriccore/logging/loggers/coutlogger.h>
#include <libriccore/logging/loggers/rnpmessagelogger.h>
#include <libriccore/logging/loggers/syslogger.h>
#include "Loggers/ControllerLogger/controllerlogger.h"
#include "Loggers/EstimatorLogger/estimatorlogger.h"
#include "Loggers/TelemetryLogger/telemetrylogger.h"


namespace RicCoreLoggingConfig
{
    enum class LOGGERS
    {
        SYS, // default system logging
        TELEMETRY,
        ESTIMATOR,
        CONTROLLER,
        COUT // cout logging
    };

    extern std::tuple<SysLogger,TelemetryLogger,EstimatorLogger,ControllerLogger,CoutLogger> logger_list;
}; 
