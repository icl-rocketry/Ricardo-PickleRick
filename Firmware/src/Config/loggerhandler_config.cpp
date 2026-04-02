#include "Config/loggerhandler_config.h"

std::tuple<SysLogger, TelemetryLogger, CoutLogger> RicCoreLoggingConfig::logger_list = {
    SysLogger(), TelemetryLogger(), CoutLogger("COUT_LOG")};