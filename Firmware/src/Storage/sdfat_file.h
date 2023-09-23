#pragma once

#include <libriccore/threading/riccorethread.h>
#include <libriccore/storage/wrappedfile.h>

#include <iostream>
#include <string>

#include <SdFat.h>

class SdFat_Store;

class SdFat_WrappedFile : public WrappedFile
{
public:
    SdFat_WrappedFile(FsFile file, store_fd fileDesc, SdFat_Store &store, FILE_MODE mode,size_t maxQueueSize);

    /**
     * @brief Get the Device lock
     * 
     * @return RicCoreThread::Lock_t& 
     */
    RicCoreThread::Lock_t& getDevLock();
    /**
     * @brief Get stream interface for the underlying file. WARNING NOT THREADSAFE, you must manually take the 
     * device lock using getDevLock, to aquire and release the lock.
     * 
     * @return Stream& 
     */
    Stream& IStream(){return _file;};

    ~SdFat_WrappedFile();

protected:
    void _read(std::vector<uint8_t> &dest) override;

    void _close() override;

private:

    /**
     * @brief Underlying File
     *
     */
    FsFile _file;

    void file_write(const std::vector<uint8_t>& data) override;

    void file_flush() override;

};