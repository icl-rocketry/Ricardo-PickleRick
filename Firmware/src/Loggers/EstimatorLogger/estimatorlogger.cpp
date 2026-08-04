#include "Loggers/EstimatorLogger/estimatorlogger.h"

EstimatorLogger::EstimatorLogger()
    : _file(nullptr),
      internalLogCB(),
      _buffer(),
      _framesBuffered(0)
{
    _buffer.reserve(EstimatorLogframe::size() * BUFFERED_FRAMES);
}

bool EstimatorLogger::initialize(std::unique_ptr<WrappedFile> file,
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
    std::string header_string = EstimatorLogframe::csvHeader();
    std::vector<uint8_t> header_bytes(header_string.begin(), header_string.end());
    _file->append(header_bytes);

    initialized = true;
    return true;
}

void EstimatorLogger::log(EstimatorLogframe& logframe)
{
    if (!initialized)
    {
        return;
    }
    if (!enabled)
    {
        return;
    }

    std::string dataframe_string = logframe.stringify();
    _buffer.insert(_buffer.end(), dataframe_string.begin(), dataframe_string.end());
    _framesBuffered++;

    if (_framesBuffered < BUFFERED_FRAMES)
    {
        return;
    }

    flushBuffer();
}

void EstimatorLogger::flushBuffer()
{
    if (_buffer.empty())
    {
        return;
    }

    try
    {
        _file->append(_buffer);
        _buffer.clear();
        _framesBuffered = 0;
    }
    catch (std::exception& e)
    {
        initialized = false;
        _buffer.clear();
        _framesBuffered = 0;
        if (internalLogCB)
        {
            internalLogCB(e.what());
        }
    }
}

void EstimatorLogger::closeFile()
{
    flushBuffer();
    if (_file)
    {
        _file->close(false);
        _file.reset();
    }
    initialized = false;
}
