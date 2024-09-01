/**
 * @file PCA9685.h
 * @author Kiran de Silva (kd619@ic.ac.uk)
 * @brief I2C pca9685 driver. //!NOTE i2c addresses 0x70 and 0x06 cannot be used with a PCA9685 on the bus
 * @version 0.1
 * @date 2024-09-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <Wire.h>
#include <atomic>

#include <libriccore/threading/riccorethread.h>


class PCA9685
{
    public:

        explicit PCA9685(TwoWire& wire,uint32_t freq = 50):
        PCA9685(0x80,wire,freq){};

        explicit PCA9685(uint8_t address,TwoWire& wire,uint32_t freq):
        m_address(address),
        m_wire(wire),
        m_freq(freq),
        mode1Shadow(0x0),// disable all call addr
        mode2Shadow()
        {};

        bool setup();

        void writeDuty(uint8_t pin, uint32_t duty);

        void writeWidth(uint8_t pin, uint32_t usec);

        bool alive();

        bool setFrequency(uint32_t freq);

        uint8_t getResolution(){return resolution;};

        uint32_t getFrequency(){return m_freq;};

    private:
        const uint8_t m_address;
        TwoWire& m_wire;
        uint32_t m_freq;

        /**
         * @brief Lock to prevent issues if multiple threads try to change shadow reigsters at the same time. 
         * Ensure i2c device lock isnt taken before calling digitalWrite or pinMode otherwise deadlock is likely
         * 
         */
        RicCoreThread::Lock_t device_lock;

        /**
         * @brief Resolution of duty cycle in bits 12bit -> 4096 steps
         * 
         */
        static constexpr uint8_t resolution = 12;

        //Register Addresses
        static constexpr uint8_t MODE1 = 0x00;
        static constexpr uint8_t MODE2 = 0x01;
        static constexpr uint8_t ALLCALLADR = 0x05;
        static constexpr uint8_t LEDSTART = 0x06;
        static constexpr uint8_t ALL_LED_ON_START = 0xFA;
        static constexpr uint8_t PRE_SCALE = 0xFE;

        //shadow registers
        uint8_t mode1Shadow;
        uint8_t mode2Shadow;
        



    private:
        //methods

        uint8_t readRegister(uint8_t reg);

        size_t writeRegister(uint8_t reg, uint8_t data);


};