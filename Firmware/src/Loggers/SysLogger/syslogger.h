/**
 * @file syslogger.h
 * @author Kiran de Silva (kd619@ic.ac.uk)
 * @brief System logger impelmentation, allows for network redirection of log messages aswell. Note if the file fails to initialize, or the
 * logger enters a non-intiialized state, the rnpmessagelogger may still be initializes and will still send log messages to the network target
 * @version 0.1
 * @date 2023-09-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <libriccore/logging/loggers/loggerbase.h>
#include <libriccore/logging/loggers/rnpmessagelogger.h>

#include <memory>
#include <string>
#include <exception>
#include <functional>
#include <string_view>


#include <libriccore/platform/millis.h>
#include <libriccore/storage/wrappedfile.h>

#include <librnp/rnp_packet.h>
#include <librnp/rnp_networkmanager.h>


class SysLogger : public LoggerBase
{
    public:
        SysLogger();

        /**
         * @brief Initalize logger using only file logger
         * 
         * @param file 
         */
        bool initialize(std::unique_ptr<WrappedFile> file, std::function<void(std::string_view message)> logcb=nullptr);
        
        /**
         * @brief Initalize both file and network logging
         * 
         * @param file 
         * @param netman 
         */
        bool initialize(std::unique_ptr<WrappedFile> file,RnpNetworkManager& netman, std::function<void(std::string_view message)> logcb=nullptr);

        /***
         * 
        */
        void log(std::string_view msg);

        void log(uint32_t status,uint32_t flag,std::string_view msg);

        void changeNetworkTarget(uint8_t destination,uint8_t destination_serivce);

    private:

        std::unique_ptr<WrappedFile> _file;

        RnpMessageLogger rnpmessagelogger;

        /**
         * @brief Log call back to allow logging of errors within the sys logger. The default implementat
         * calls the current instance log function so that if the network logger is setup, we still capture 
         * file errors. Can be overriden in the intializer.
         * 
         */
        std::function<void(std::string_view message)> internalLogCB = [this](std::string_view message){log(message);};

        void writeLogString(uint32_t status,uint32_t flag,std::string_view msg);

        

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
