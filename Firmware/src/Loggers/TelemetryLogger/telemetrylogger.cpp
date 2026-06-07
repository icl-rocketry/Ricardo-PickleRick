#include "Loggers/TelemetryLogger/telemetrylogger.h"

TelemetryLogger::TelemetryLogger():
_file(nullptr),
internalLogCB()
{};

bool TelemetryLogger::initialize(std::unique_ptr<WrappedFile> file,std::function<void(std::string_view message)> logcb)
{
    if (logcb)
    {
        internalLogCB = logcb;
    }

    if (file == nullptr){return false;};
    _file = std::move(file);
    std::string header_string = TelemetryLogframe::csvHeader();
    std::vector<uint8_t> header_bytes(header_string.begin(),header_string.end());
    _file->append(header_bytes);

    initialized=true;
    return true;
}

void TelemetryLogger::log(TelemetryLogframe& logframe)
{
    if (!initialized){return;};
    if (!enabled){return;};

    std::string dataframe_string = logframe.stringify();

    std::vector<uint8_t> dataframe_bytes(dataframe_string.begin(),dataframe_string.end());

    //if there is any exception we want to force the user to re-initialize the file, as a wrapped file will
    //automatically close itself when an exception is thrown
    try{
        _file->append(dataframe_bytes);
    }
    catch(std::exception &e)
    {
        initialized=false;
        internalLogCB(e.what());
    }

}
