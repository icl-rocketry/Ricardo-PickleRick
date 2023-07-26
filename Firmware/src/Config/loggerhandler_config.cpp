#include "Config/loggerhandler_config.h"

#include <libriccore/logging/loggers/coutlogger.h>
#include <libriccore/logging/loggers/rnpmessagelogger.h>

std::tuple<RnpMessageLogger,CoutLogger> RicCoreLoggingConfig::logger_list =
{
    RnpMessageLogger("SYS_LOG",1),
    CoutLogger("COUT_LOG")
};