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

#include "Loggers/EstimatorLogger/estimatorlogframe.h"

class EstimatorLogger : public LoggerBase
{
public:
    EstimatorLogger();

    bool initialize(std::unique_ptr<WrappedFile> file,
                    std::function<void(std::string_view message)> logcb = nullptr);

    void log(EstimatorLogframe& logframe);
    void closeFile();

private:
    static constexpr size_t BUFFERED_FRAMES = 20;

    std::unique_ptr<WrappedFile> _file;
    std::function<void(std::string_view message)> internalLogCB;
    std::vector<uint8_t> _buffer;
    size_t _framesBuffered;

    void flushBuffer();

public:
    class LogException : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };
};
