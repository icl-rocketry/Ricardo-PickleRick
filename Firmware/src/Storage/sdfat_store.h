#pragma once
/**
 * @file sdfat_store.h
 * @author Kiran de Silva (kd619@ic.ac.uk)
 * @brief Sdfat store, supports fat and exfat
 * @version 0.1
 * @date 2023-08-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <libriccore/storage/storebase.h>

#include <libriccore/threading/riccorethread.h>

#include <SPI.h>
#include <SdFat.h>

#include "Config/types.h"



class SdFat_Store : public StoreBase
{
    public:
        SdFat_Store(SPIClass &spi,const uint8_t cs,const uint32_t frequency,RicCoreThread::Lock_t &spiBusLock,bool dedicatedSPI,Types::CoreTypes::SystemStatus_t* systemstatus);
        SdFat_Store(SPIClass &spi,const uint8_t cs, const uint32_t frequency,bool dedicatedSPI,Types::CoreTypes::SystemStatus_t* systemstatus);

        void setup();

        ~SdFat_Store(){};
    protected:
         std::unique_ptr<WrappedFile> _open(std::string_view path, store_fd fileDesc, FILE_MODE mode, size_t maxQueueSize) override;
         bool _ls(std::string_view path, std::vector<directory_element_t> &directory_structure) override;
         bool _mkdir(std::string_view path) override;
         bool _remove(std::string_view path) override;


    private:
        /**
         * @brief Reference to spi bus
         * 
         */
        SPIClass &_spi;

        /**
         * @brief SD fat config
         * 
         */
        const SdSpiConfig _config;

        /**
         * @brief dummy lock if the store does not require a device lock i.e it is the only device on the bus
         * 
         */
        RicCoreThread::Lock_t dummyLock;

        Types::CoreTypes::SystemStatus_t* _systemstatus;

        /**
         * @brief underlying file system - supports both fat and exfat
         * 
         */
        SdFs filesys;

        // enum class STATUS:uint8_t{
        //     NOMINAL,
        //     ERROR_INIT,
        //     ERROR_OPEN
        // };

        // void reportStatus(std::string_view message, bool raiseError);



};