#include "syslogger.h"

#include <libriccore/logging/loggers/loggerbase.h>
#include <libriccore/logging/loggers/rnpmessagelogger.h>

#include <memory>
#include <string>
#include <exception>
#include <string_view>


#include <libriccore/platform/millis.h>
#include <libriccore/storage/wrappedfile.h>

#include <librnp/rnp_packet.h>
#include <librnp/rnp_networkmanager.h>

SysLogger::SysLogger():
_file(nullptr),
rnpmessagelogger("")
{};

bool SysLogger::initialize(std::unique_ptr<WrappedFile> file,std::function<void(std::string_view message)> logcb)
{
    if (logcb)
    {
        //update internal logging call back if a valid logging callback has been passed in
        internalLogCB = logcb;
    }

    if (file == nullptr){return false;};
    _file = std::move(file);
    initialized = true;
    return true;
};

bool SysLogger::initialize(std::unique_ptr<WrappedFile> file,RnpNetworkManager& netman,std::function<void(std::string_view message)> logcb)
{
    rnpmessagelogger.initialize(netman);
    return initialize(std::move(file),logcb); 
};

void SysLogger::log(std::string_view msg)
{
    rnpmessagelogger.log(msg);
    writeLogString(0,0,msg);
};

void SysLogger::log(uint32_t status,uint32_t flag,std::string_view msg)
{
    if (!enabled){return;};
    rnpmessagelogger.log(status,flag,msg);
    writeLogString(status,flag,msg);
};

void SysLogger::changeNetworkTarget(uint8_t destination,uint8_t destination_service)
{
    if (!enabled){return;};
    rnpmessagelogger.changeNetworkTarget(destination,destination_service);
};

void SysLogger::writeLogString(uint32_t status,uint32_t flag,std::string_view msg)
{
    if (!initialized){return;};
    
    //construct data frame to write to file
    std::string dataframe_string = std::to_string(millis()) + "," + std::string(msg) + "," + std::to_string(flag) + "," + std::to_string(status) + ",\n";

    std::vector<uint8_t> dataframe_bytes(dataframe_string.begin(),dataframe_string.end());

    //if there is an exception, we want hte user to re-intialize the logger.
    try{
        _file->append(dataframe_bytes);
    }
    catch(std::exception &e)
    {
        initialized=false;
        internalLogCB(e.what()); // calling this after intialized is false will still allow the internal network logger to be called
        // Serial.println(e.what());
    }
    
};