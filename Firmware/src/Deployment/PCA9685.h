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
        mode1Shadow(mode1Default),// disable all call addr
        mode2Shadow(mode2Default)
        {};

        bool setup();

        /**
         * @brief Write duty cycle [0,4096]
         * 
         * @param pin 
         * @param duty 
         */
        void writeDuty(uint8_t pin, uint32_t duty);

        void writeWidth(uint8_t pin, uint32_t usec);

        bool alive();

        void setFrequency(uint32_t freq);

        uint8_t getResolution(){return resolution;};

        uint32_t getFrequency(){return m_freq;};

    private:
        const uint8_t m_address;
        TwoWire& m_wire;
        uint32_t m_freq;

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
        static constexpr uint8_t ALL_LED_START = 0xFA;
        static constexpr uint8_t PRE_SCALE = 0xFE;

        //shadow registers
        uint8_t mode1Shadow;
        uint8_t mode2Shadow;

        /**
         * @brief Default state of mode1 register
         * Restart disabled
         * No Ext Clk
         * Auto increment enabled
         * Normal Mode
         * Sub1 disabled
         * Sub2 disabled
         * Sub3 disabled
         * Allcall i2c address disabled
         * 
         */
        static constexpr uint8_t mode1Default = 0b00100000; //everything 0
        static constexpr uint8_t SLEEP = 1 << 4;
        static constexpr uint8_t RESTART = 1 << 7;
        static constexpr uint8_t AUTO_INC = 1 << 5;


        /**
         * @brief Defaault staet of mode2 register
         *  no invert -> 0
         *  output change on I2C STOP -> 0
         *  totem pole to drive output high and low -> 0
         *  OE -> 0
         */
        static constexpr uint8_t mode2Default = 0x00; 

        static constexpr uint8_t MAX_CHANNEL = 16;
        
        /**
         * @brief internal oscilator freuqnecy at 25mhz
         * 
         */
        static constexpr int OSC_FREQUENCY = 25e6;



    private:
        //methods

        uint8_t readRegister(uint8_t reg);

        size_t writeRegister(uint8_t reg, uint8_t data);

        uint32_t toCounts(uint32_t usec);

        void writeTiming(uint8_t startAddress,uint16_t on,uint16_t off);


};