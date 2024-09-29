#pragma once

#include <Wire.h>
#include <cmath>
#include <libriccore/riccorelogging.h>

class INA219
{
public:
    INA219(uint8_t I2Caddr, TwoWire &I2CObj) : _deviceAddr(I2Caddr),
                                               _wire(I2CObj) {};

    // register addresses
    enum class Registers : uint8_t
    {
        Config = 0x00,
        ShuntV = 0x01,
        BusV = 0x02,
        Power = 0x03,
        Current = 0x04,
        Calibration = 0x05,
    };

    // config register settings
    enum class PGAGain : uint8_t // left shift by 11 bits in gain set method
    {
        gain1 = 0b00,
        gain2 = 0b01,
        gain4 = 0b10,
        gain8 = 0b11
    };

    enum class ADCSettings : uint8_t // left shift by different amount for BADC and SADC.
    {
        res9bit = 0b0000,
        res10bit = 0b0001,
        res11bit = 0b0010,
        res12bit = 0b0011,
        avrg2samp = 0b1001,
        avrg4samp = 0b1010,
        avrg8samp = 0b1011,
        avrg16samp = 0b1100,
        avrg32samp = 0b1101,
        avrg64samp = 0b1110,
        avrg128samp = 0b1111
    };

    enum class Modes : uint8_t // no left shift required, last 3 bits of the config register
    {
        PowerDown = 0b000,
        ShuntVTrig = 0b001,
        BusVTrig = 0b010,
        ShuntandBusTrig = 0b011,
        ADCOFF = 0b100,
        ShuntVConti = 0b101,
        BusVConti = 0b110,
        ShuntandBusConti = 0b111
    };

    enum class busVRange : bool
    {
        FSR16V = 0,
        FSR32V
    };

    void setup(float resistance, float maxCurrent, PGAGain Gain = PGAGain::gain2, ADCSettings ShuntVADCsettings = ADCSettings::avrg64samp, ADCSettings BusVADCsettings = ADCSettings::avrg128samp, Modes DeviceMode = Modes::ShuntandBusConti, busVRange vRange = busVRange::FSR32V);

    void update();

    float getCurrent() { return _current; };
    float getBusV() { return _busV; };
    float getPower() { return _power; };

    void setPollDelta(uint32_t polldelta) { _pollDelta = polldelta; };

    int16_t calibreg;
    int16_t configreg;

private:
    //  methods
    void writeRegister(INA219::Registers address, uint16_t value);
    int16_t readRegister(INA219::Registers address);

    // device settings
    void setCalibration(float resistance, float maxCurrent);
    void setBusVRange(INA219::busVRange range);
    void setGain(INA219::PGAGain Gain);
    void setShuntVADC(INA219::ADCSettings ADCsetting);
    void setBusVADC(INA219::ADCSettings ADCsetting);
    void setMode(INA219::Modes Mode);
    void reset();

    // shadow registers, initialised to default value
    uint16_t _configReg = 0x399F;
    uint16_t _calibrationReg = 0x0000;

    // register masks
    static constexpr uint16_t busVRangeMask = 0b1101111111111111;
    static constexpr uint16_t gainMask = 0b1110011111111111;
    static constexpr uint16_t busADCMask = 0b1111100001111111;
    static constexpr uint16_t shuntADCMask = 0b1111111110000111;
    static constexpr uint16_t modeMask = 0b1111111111111000;
    static constexpr uint16_t resetMask = 0b0111111111111111;

    static constexpr uint16_t signmask16bit = 0b1000000000000000;
    // I2C vars
    uint8_t _deviceAddr;
    TwoWire &_wire;

    // calibration consts
    float _currentLSB;

    // sensor readings
    float _current;
    float _busV;
    float _power;

    // magic numbers
    static constexpr float recip2pow15 = 1.0 / 32768.0;
    static constexpr float busVLSB = 4e-3; // 4 mV

    // timing
    uint32_t _prevUpdate = 0;
    uint32_t _pollDelta = 50;
};