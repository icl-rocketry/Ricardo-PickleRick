#pragma once

#include <tuple>

#include <libriccore/logging/loggers/coutlogger.h>
#include <libriccore/logging/loggers/rnpmessagelogger.h>

namespace RicCoreLoggingConfig
{
    enum class LOGGERS
    {
        SYS, // default system logging
        COUT // cout logging
    };

    extern std::tuple<RnpMessageLogger,CoutLogger> logger_list;
}; 


