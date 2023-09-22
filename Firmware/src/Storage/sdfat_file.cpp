#include "sdfat_file.h"

#include <SdFat.h>
#include <libriccore/storage/wrappedfile.h>

#include "sdfat_store.h"



SdFat_WrappedFile::SdFat_WrappedFile(FsFile file, SdFat_Store& store, FILE_MODE mode,size_t maxQueueSize):
WrappedFile(store,mode,maxQueueSize),
_file(file)
{}

RicCoreThread::Lock_t& SdFat_WrappedFile::getDevLock()
{
    return store.get_lock();
};

void SdFat_WrappedFile::_read(std::vector<uint8_t> &dest) 
{
    size_t numBytes = _file.read(dest.data(), dest.size());
    if(numBytes == -1)
    {
        //could be io error or read called when *underlying* file not opened, idk if we can differentiate this
        throw WrappedFile::ReadException();
    }
}

void SdFat_WrappedFile::_close() 
{
    if(!_file.close())
    {
        throw WrappedFile::CloseException();
    }
}

void SdFat_WrappedFile::file_write(const std::vector<uint8_t>& data)
{

    size_t numBytes = _file.write(data.data(), data.size());
    if (numBytes == 0)
    {
        if (_file.getWriteError())
        {
            throw WrappedFile::WriteException();
        }
    }
}

void SdFat_WrappedFile::file_flush() 
{
    if(!_file.sync())
    {
        throw WrappedFile::FlushException();
    }
}

SdFat_WrappedFile::~SdFat_WrappedFile()
{
    _close();
}