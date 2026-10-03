#include "Config/loggerhandler_config.h"

#include <libriccore/logging/loggers/coutlogger.h>
#include <libriccore/logging/loggers/rnpmessagelogger.h>
#include <libriccore/logging/loggers/syslogger.h>
#include "Loggers/TelemetryLogger/telemetrylogger.h"
#include "Loggers/ApogeeLogger/apogeelogger.h"

std::tuple<SysLogger,TelemetryLogger,ApogeeLogger,CoutLogger> RicCoreLoggingConfig::logger_list =
{
    SysLogger(),
    TelemetryLogger(),
    ApogeeLogger(),
    CoutLogger("COUT_LOG")
};
