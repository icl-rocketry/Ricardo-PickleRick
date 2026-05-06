#include "Config/loggerhandler_config.h"

std::tuple<SysLogger, TelemetryLogger, EstimatorLogger, CoutLogger> RicCoreLoggingConfig::logger_list = {
    SysLogger(), TelemetryLogger(), EstimatorLogger(), CoutLogger("COUT_LOG")};
