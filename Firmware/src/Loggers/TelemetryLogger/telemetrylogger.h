#pragma once

#include <libriccore/logging/loggers/loggerbase.h>

#include <memory>
#include <string>
#include <string_view>
#include <exception>
#include <functional>


#include <libriccore/storage/wrappedfile.h>

#include "telemetrylogframe.h"



class TelemetryLogger : public LoggerBase
{
    public:
        TelemetryLogger();

        /**
         * @brief Initializes logger with given file. Returns true on success
         * 
         * @param file 
         * @return true 
         * @return false 
         */
        bool initialize(std::unique_ptr<WrappedFile> file,std::function<void(std::string_view message)> logcb=nullptr);

        /**
         * @brief Logs given logframe as a string to the provided file
         * 
         * @param logframe 
         */
        void log(TelemetryLogframe& logframe);

    private:

        /**
         * @brief Pointer to log file 
         * 
         */
        std::unique_ptr<WrappedFile> _file;

        /**
         * @brief logging callback to log any errors with writing to file
         * 
         */
        std::function<void(std::string_view message)> internalLogCB;

    public:

        /**
         * @brief Define custom excpetion which inherits directly from runtime error to support custom messages
         * 
         */
        class LogException : public std::runtime_error
        {
            public:
                using std::runtime_error::runtime_error;
        };

};
