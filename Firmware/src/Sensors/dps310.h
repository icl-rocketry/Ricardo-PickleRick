#pragma once

#include <SPI.h>
#include <Dps3xx.h>

#include "Config/types.h"

#include "sensorStructs.h"

class DPS310 : private Dps3xx{

    public:
        DPS310(SPIClass& spi, Types::CoreTypes::SystemStatus_t& systemstatus,uint8_t cs);

        void setup();

        void update(SensorStructs::BARO_t &data);

        void calibrateBaro();

    private:
        SPIClass & _spi;


        Types::CoreTypes::SystemStatus_t& _systemstatus;
        const uint8_t _cs;
        bool _initialized;

        /**
         * @brief Reference pressure and temp for altitude calculation
         * 
         */
        float refTemp{273.15 + 15};
        float refPress{101325};

        /**
         * @brief This reads the most recent pressure and temperautre result in the result registers.
         * 
         * @param pressure 
         * @param temp 
         */
        void readDPS(float& pressure,float& temperature);

        
        /**
         * @brief  Temperature Measurement rate 
         * 
         */
        static constexpr int temp_mr = 3;
        /**
         * @brief Temperature Over sampling rate
         * 
         */
        static constexpr int temp_osr = 1;
        /**
         * @brief Pressure Measurement Rate
         * 
         */
        static constexpr int press_mr = 6;
        /**
         * @brief Pressure Over sampling rate
         * 
         */
        static constexpr int press_osr = 2;

        float toAltitude(const float& pressure);

};