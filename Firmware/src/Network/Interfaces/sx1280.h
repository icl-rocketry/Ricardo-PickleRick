#pragma once
// SX1280 Driver

#include <SPI.h>
#include <vector>
#include <Arduino.h> 
#include <cmath>
#include <type_traits>

class sx1280 :public Stream{
public:

// Constructor

sx1280();
// Commands

// SPI Interface
// template <typename T, typename... Args> std::vector<uint8_t> singleTransfer(T first, Args... args);

void beginSPI();
void endSPI();
void setPins(int ss, int reset, int dio0);
void setSPI(SPIClass& spi);
void GetStatus(); //Not needed for SPI interface

// Register Access
void WriteRegister(uint16_t address, uint32_t value); 
void ReadRegister(uint16_t address);

// Data Buffer
void WriteBuffer(uint8_t &offset, std::vector<uint8_t> &payload); //Used to write the data payload to be transmitted
void ReadBuffer(uint8_t &offset);

// Radio Operations
void SetSleep(int mode);
void SetStandby(int mode);
void SetFs(); //Frequency Synthesiser mode
void SetTx(int periodBase);
void SetRx(int periodBase, int timeoutMode);
void SetLongPreamble(); //In RxDutyCycle mode extends Rx window
void SetRxDutyCycle(int periodBase); //Command SetLongPreamble must be issued before this command
void SetCad(); //Channel Activity Detection
void SetTxContinuousWave(); //Test command at specified frequency
void SetTxContinousPreamble(); //Test command with LoRa symbol 0
void SetAutoTx();
void SetAutoFs(int enable); 

// Radio Configuration
void SetPacketType(int packetType);
void GetPacketType();
void SetRfFrequency(uint32_t frequency); //Call after setPacketType()
void SetTxParams(int power, int ramptime); 
void SetCadParams(uint8_t cadSymbolNum); 
void SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);
void SetModulationParams(uint8_t modParam1, uint8_t modParam2, uint8_t modParam3);
void SetPacketParams(uint8_t packetParam1, uint8_t packetParam2, uint8_t packetParam3, uint8_t packetParam4, uint8_t packetParam5);
void SetDioIrqParams();

// Communication Status Information
void GetRxBufferStatus(); //returns last received packet length and address of first byte received
void GetPacketStatus();
int8_t GetRssiInst(); //Get instantaneous RSSI
int8_t GetSnrPkt(){GetPacketStatus();return snrPkt;}


// Interrupt Request Handling
void GetIrqStatus();
void ClearIrqStatus(uint16_t _irqFlag);
void SetRegulatorMode();
void SetSaveContext();

// Ranging Operation
void SetRangingRole();
void SetAdvancedRanging();

// get number of bytes
uint8_t size(uint32_t value){return ceil(log2(value));};

uint8_t getoffset(){return _offset;};

// LoRa setup

void setup();
void txSetup(std::vector<uint8_t> &data);
void rxSetup();

int parsePacket();



template <typename T>
    std::vector<uint8_t> singleTransfer(T value) {
        std::vector<uint8_t> response;
        for (uint8_t i = 0; i < sizeof(T); i++) {
            response.push_back(_spi->transfer(static_cast<uint8_t>((value >> (i * 8)) & 0xFF)));
        }
        return response;
    }


template <typename T, typename... Args>
    std::vector<uint8_t> singleTransfer(T first, Args... args) {
        digitalWrite(_ss, LOW);
        _spi->beginTransaction(_spiSettings);
        std::vector<uint8_t> response = (singleTransfer(first, args...));
        _spi->endTransaction();
        digitalWrite(_ss, HIGH);

        return response;
    }

    




// JANK IGNORE (Stream ovverides)

int available(){return 0;};
int read(){return 0;};
int peek(){return 0;};
size_t write(uint8_t x){return 0;};
// 


SPISettings _spiSettings;
SPIClass* _spi;
uint8_t _ss;
private:

    
    int _reset;
    int _dio0;
    long _frequency;
    int _packetIndex;
    int _implicitHeaderMode;
    void (*_onReceive)(int);
    void (*_onCadDone)(boolean);
    void (*_onTxDone)();

    uint8_t _txBaseAddress = 0x00;
    uint8_t _rxBaseAddress = 0x80;
    uint8_t _packetType;
    uint8_t _rxPayloadLength;
    uint8_t _rxStartBufferPointer;
    uint8_t _rssiSync;
    int8_t snrPkt;
    int8_t _rssiInst;
    uint16_t _irqstatus;
    uint8_t _offset = 0;
    std::vector<uint8_t> _data;
    uint8_t _register;
    uint8_t _buffer;

    std::vector<uint8_t> spiReturn;


};