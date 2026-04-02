#pragma once

#include <Arduino.h>
#include <SPI.h>

#include <libriccore/riccorelogging.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include "Sensors/sensorStructs.h"


class DPS368
{
    public:
        DPS368(SPIClass& spi, Types::CoreTypes::SystemStatus_t& systemstatus, uint8_t cs);

        void setup();
        void update(SensorStructs::BARO_t& data);

    private:

        SPIClass& _spi;
        Types::CoreTypes::SystemStatus_t& _systemstatus;
        const uint8_t _cs;
        SPISettings _settings;

        // Calibration coefficients
        int32_t  _c0Half, _c1;
        int32_t  _c00, _c10, _c01, _c11, _c20, _c21, _c30;

        // Scale factors chosen to match our oversampling config (16x standard)
        static constexpr float kT = 524288.0f;
        static constexpr float kP = 253952.0f;

        float _lastTraw_sc = 0.0f;  // seeded by blocking temp measurement in setup()

        bool alive();
        void readCalibrationCoefficients();

        void readPressureRaw(int32_t& raw);
        void readTemperatureRaw(int32_t& raw);

        float compensateTemperature(int32_t raw);
        float compensatePressure(int32_t rawP, float Traw_sc);

        uint8_t readRegister(uint8_t reg);
        void    writeRegister(uint8_t reg, uint8_t val);
        void    readRegisters(uint8_t reg, uint8_t* buf, uint8_t len);

        void getTwosComplement(int32_t *raw, uint8_t length);

        // -----------------------------------------------------------------------
        // Register map
        // -----------------------------------------------------------------------
        static constexpr uint8_t PSR_B2    = 0x00;   // pressure MSB
        static constexpr uint8_t TMP_B2    = 0x03;   // temperature MSB
        static constexpr uint8_t PRS_CFG   = 0x06;   // pressure config
        static constexpr uint8_t TMP_CFG   = 0x07;   // temperature config
        static constexpr uint8_t MEAS_CFG  = 0x08;   // measurement mode / status
        static constexpr uint8_t CFG_REG   = 0x09;   // interrupt / FIFO / shift
        static constexpr uint8_t RESET_REG = 0x0C;   // soft reset
        static constexpr uint8_t PROD_ID   = 0x0D;   // product / revision ID
        static constexpr uint8_t COEF_BASE = 0x10;   // first calibration coefficient register
        static constexpr uint8_t COEF_SRCE = 0x28;   // coefficient source

        // Expected product ID (lower nibble)
        static constexpr uint8_t PROD_ID_VAL = 0x10;

        // MEAS_CFG bit masks
        static constexpr uint8_t COEF_RDY   = 0x80;
        static constexpr uint8_t SENSOR_RDY  = 0x40;
        static constexpr uint8_t MEAS_IDLE    = 0x00;
        static constexpr uint8_t MEAS_CMD_TEMP= 0x02;  // single temperature measurement
        static constexpr uint8_t MEAS_CONT_PT = 0x07;  // continuous pressure + temperature

        // Pressure oversampling: 16x standard (0x04), requires P_SHIFT
        static constexpr uint8_t PM_RATE_1HZ  = 0x00; // 1 meas/sec
        static constexpr uint8_t PM_PRC_16X   = 0x04; // 16x oversampling

        // Temperature: 1x, external MEMS sensor bit set after reading COEF_SRCE
        static constexpr uint8_t TMP_PRC_1X   = 0x00;

        // CFG_REG: enable P_SHIFT and T_SHIFT for 16x oversampling
        static constexpr uint8_t P_SHIFT_EN   = 0x04;
        static constexpr uint8_t T_SHIFT_EN   = 0x08;

        // Soft reset value
        static constexpr uint8_t SOFT_RST_VAL = 0x09;
};