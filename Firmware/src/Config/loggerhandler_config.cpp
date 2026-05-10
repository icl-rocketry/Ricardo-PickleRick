#include "Config/loggerhandler_config.h"

std::tuple<SysLogger, TelemetryLogger, EstimatorLogger, ControllerLogger, CoutLogger> RicCoreLoggingConfig::logger_list = {
    SysLogger(), TelemetryLogger(), EstimatorLogger(), ControllerLogger(), CoutLogger("COUT_LOG")};
