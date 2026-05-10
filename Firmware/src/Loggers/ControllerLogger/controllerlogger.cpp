#include "Loggers/ControllerLogger/controllerlogger.h"

ControllerLogger::ControllerLogger()
    : _file(nullptr),
      internalLogCB()
{};

bool ControllerLogger::initialize(std::unique_ptr<WrappedFile> file,
                                  std::function<void(std::string_view message)> logcb)
{
    if (logcb)
    {
        internalLogCB = logcb;
    }

    if (file == nullptr)
    {
        return false;
    }

    _file = std::move(file);
    initialized = true;
    return true;
}

void ControllerLogger::log(ControllerTelemetryPacket& packet)
{
    if (!initialized)
    {
        return;
    }
    if (!enabled)
    {
        return;
    }

    std::string dataframe_string = packet.stringify();
    std::vector<uint8_t> dataframe_bytes(dataframe_string.begin(), dataframe_string.end());

    try
    {
        _file->append(dataframe_bytes);
    }
    catch (std::exception& e)
    {
        initialized = false;
        if (internalLogCB)
        {
            internalLogCB(e.what());
        }
    }
}
