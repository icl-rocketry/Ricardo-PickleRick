#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <array>
#include <string>

#include <libriccore/riccorelogging.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include "Helpers/axeshelper.h"
#include "Sensors/sensorStructs.h"


class ICM_20608{
    public:
        ICM_20608(SPIClass& spi,Types::CoreTypes::SystemStatus_t& systemstatus,uint8_t cs);

        void setup(const std::array<uint8_t,3>& axesOrder, const std::array<bool,3> axesFlip);
        void update(SensorStructs::ACCELGYRO_6AXIS_t& data);

    private:

        SPIClass& _spi;
        Types::CoreTypes::SystemStatus_t& _systemstatus;
        const uint8_t _cs;
        SPISettings _settings;
        AxesHelper<> axeshelper;

        enum GyroRange:uint8_t
        {
            G_250_DEGS,
            G_500_DEGS,
            G_1000_DEGS,
            G_2000_DEGS
        };

        enum AccelRange:uint8_t
        {
            A_2_G,
            A_4_G,
            A_8_G,
            A_16_G
        };

        bool alive();
        void setRange(AccelRange accel_range,GyroRange gyro_range);

        void readGyro(float &x, float &y, float &z);
        void readAccel(float &x, float &y, float &z);
        void readTemp(float& temp);
        void readGyroRaw(int16_t &x, int16_t &y, int16_t &z);
        void readAccelRaw(int16_t &x, int16_t &y, int16_t &z);
        void readTempRaw(int16_t& temp);
        uint8_t readRegister(uint8_t reg);
        void writeRegister(uint8_t reg, uint8_t val);

        float gyro_lsb_to_degs;
        float accel_lsb_to_g;

        static constexpr float temperature_sensitivity = 326.8f;

        //registers
        static constexpr uint8_t WHO_AM_I = 0x75;
        static constexpr uint8_t WHO_AM_I_RES = 0xAF; //this changes dependign on the chip

        static constexpr uint8_t CONFIG = 0x1A;

        static constexpr uint8_t GYRO_CONFIG = 0x1B;
        static constexpr uint8_t DPS250 = 0b00000000;
        static constexpr uint8_t DPS500 = 0b00001000;
        static constexpr uint8_t DPS1000 = 0b00010000;
        static constexpr uint8_t DPS2000 = 0b00011000;

        static constexpr uint8_t ACCEL_CONFIG = 0x1C;
        static constexpr uint8_t G2 = 0b00000000;
        static constexpr uint8_t G4 = 0b00001000;
        static constexpr uint8_t G8 = 0b00010000;
        static constexpr uint8_t G16 = 0b00011000;

        static constexpr uint8_t ACCEL_CONFIG2 = 0x1D;

        static constexpr uint8_t USER_CTRL = 0x6A;
        static constexpr uint8_t I2C_IF_DIS = 0b00010000;
        static constexpr uint8_t FIFO_EN = 0b01000000;

        static constexpr uint8_t PWR_MGMT_1 = 0x6B;
        static constexpr uint8_t SLEEP = 0b01000000;
        static constexpr uint8_t RESET = 0b10000000;
        static constexpr uint8_t CLK_ZGYRO = 0b00000011;

        static constexpr uint8_t PWR_MGMT_2 = 0x6C;

        static constexpr uint8_t ACCEL_XOUT_H = 0x3B;
        static constexpr uint8_t ACCEL_XOUT_L = 0x3C;
        static constexpr uint8_t ACCEL_YOUT_H = 0x3D;
        static constexpr uint8_t ACCEL_YOUT_L = 0x3E;
        static constexpr uint8_t ACCEL_ZOUT_H = 0x3F;
        static constexpr uint8_t ACCEL_ZOUT_L = 0x40;
        static constexpr uint8_t TEMP_OUT_H = 0x41;
        static constexpr uint8_t TEMP_OUT_L = 0x42;
        static constexpr uint8_t GYRO_XOUT_H = 0x43;
        static constexpr uint8_t GYRO_XOUT_L = 0x44;
        static constexpr uint8_t GYRO_YOUT_H = 0x45;
        static constexpr uint8_t GYRO_YOUT_L = 0x46;
        static constexpr uint8_t GYRO_ZOUT_H = 0x47;
        static constexpr uint8_t GYRO_ZOUT_L = 0x48;

        static constexpr uint8_t XG_OFFS_USRH = 0x13;
        static constexpr uint8_t XG_OFFS_USRL = 0x14;
        static constexpr uint8_t YG_OFFS_USRH = 0x15;
        static constexpr uint8_t YG_OFFS_USRL = 0x16;
        static constexpr uint8_t ZG_OFFS_USRH = 0x17;
        static constexpr uint8_t ZG_OFFS_USRL = 0x18;
        static constexpr uint8_t XA_OFFSET_H = 0x77;
        static constexpr uint8_t XA_OFFSET_L = 0x78;
        static constexpr uint8_t YA_OFFSET_H = 0x7A;
        static constexpr uint8_t YA_OFFSET_L = 0x7B;
        static constexpr uint8_t ZA_OFFSET_H = 0x7D;
        static constexpr uint8_t ZA_OFFSET_L = 0x7E;

        static constexpr uint8_t UNDOC1 = 0x11;
        static constexpr uint8_t UNDOC1_VALUE = 0xc9;
};