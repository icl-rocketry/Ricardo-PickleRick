#pragma once

#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <libriccore/logging/loggers/loggerbase.h>
#include <libriccore/storage/wrappedfile.h>

#include "GNC/ControllerTelemetryPacket.h"

class ControllerLogger : public LoggerBase
{
public:
    ControllerLogger();

    bool initialize(std::unique_ptr<WrappedFile> file,
                    std::function<void(std::string_view message)> logcb = nullptr);

    void log(ControllerTelemetryPacket& packet);
    void closeFile();

private:
    std::unique_ptr<WrappedFile> _file;
    std::function<void(std::string_view message)> internalLogCB;

public:
    class LogException : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };
};
