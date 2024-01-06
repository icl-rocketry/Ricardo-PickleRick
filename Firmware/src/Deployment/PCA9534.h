#pragma once

#include <stdint.h>

#include <Wire.h>
#include <atomic>

#include <libriccore/threading/riccorethread.h>


class PCA9534
{
    public:
    
        enum class PINMODE:uint8_t {
            GPIO_OUTPUT = 0,
            GPIO_INPUT = 1,
            GPIO_INPUT_INVERT = 2
        };

    public:

        PCA9534(TwoWire& wire):
        PCA9534(0x20,wire)
        {};

        PCA9534(uint16_t address,TwoWire& wire):
        m_address(address),
        m_wire(wire),
        outputShadow(0x0), // default off
        polarityShadow(0x0), // default no inversion
        configShadow(0xff) //default all inputs
        {};

        
        void pinMode(uint8_t pin, PINMODE mode);

        void digitalWrite(uint8_t pin, uint8_t level);

        int digitalRead(uint8_t pin);

        bool alive();

    private:

        /**
         * @brief Get the value in register
         * 
         * @param reg 
         * @return uint8_t 
         */
        uint8_t readRegister(uint8_t reg);

        /**
         * @brief Write data to specified register
         * 
         * @param reg register to write to
         * @param data data to write
         * @return size_t num bytes written
         */
        size_t writeRegister(uint8_t reg, uint8_t data);


    private:
        const uint16_t m_address;
        TwoWire& m_wire;

        /**
         * @brief Lock to prevent issues if multiple threads try to change shadow reigsters at the same time. 
         * Ensure i2c device lock isnt taken before calling digitalWrite or pinMode otherwise deadlock is likely
         * 
         */
        RicCoreThread::Lock_t device_lock;


        //shadow registers
        uint8_t outputShadow;
        uint8_t polarityShadow;
        uint8_t configShadow;


        static constexpr uint8_t INPUT_PORT = 0x00;
        static constexpr uint8_t OUTPUT_PORT = 0x01;
        static constexpr uint8_t POLARITY = 0x02;
        static constexpr uint8_t CONFIG = 0x03;

};