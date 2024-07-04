// SX1280 Driver

#include <SPI.h>
#include <Arduino.h> 
#include <cmath>

class sx1280: public Stream {
public:

// Constructor
sx1280();

// Commands

// SPI Interface

uint32_t singleTransfer(uint8_t opcode, uint16_t address, uint32_t value);

template <typename T, typename... Args> void singleTransfer(T first, Args... args);

int beginSPI(uint32_t SPIFrequency);
void endSPI();

void GetStatus(); //Not needed for SPI interface

// Register Access
void WriteRegister(uint16_t address, uint32_t value); 
void ReadRegister(uint16_t address);

// Data Buffer
void WriteBuffer(uint8_t &offset, uint32_t value); //Used to write the data payload to be transmitted
void ReadBuffer(uint8_t &offset);

// Radio Operations
void SetSleep(int mode);
void SetStandby(int mode);
void SetFs();
void SetTx();
void SetRx();
void SetLongPreamble();
void SetRxDutyCycle(); //Command SetLongPreamble must be issued before this command
void SetCad();
void SetTxContinuousWave(); 
void SetTxContinousPreamble();
void SetAutoTx();
void SetAutoFs(); 

// Radio Configuration
void SetPacketType();
void GetPacketType();
void SetRfFrequency();
void SetTxParams(); // (opcode, power, rampTime)
void SetCadParams(); //(opcode, CAD_params)
void SetBufferBaseAddress();
void SetModulationParams();
void SetPacketParams();
void SetDioIqrParams();

// Communication Status Information
void GetRxBufferStatus(); //returns last received packet length and address of first byte received
void GetPacketStatus();
void GetRssiInst(); //Get instantaneous RSSI

// Interrupt Request Handling
void GetIqrStatus();
void ClearIqrStatus();
void SetRegulatorMode();
void SetSaveContext();

// Ranging Operation
void SetRangingRole();
void SetAdvancedRanging();

// get number of bytes
uint8_t size(uint32_t value){return ceil(log2(value));};

// LoRa setup

void setup();
void settings();
void txSetup();
void rxSetup();

private:

    SPISettings _spiSettings;
    SPIClass* _spi;
    int _ss;
    int _reset;
    int _dio0;
    long _frequency;
    int _packetIndex;
    int _implicitHeaderMode;
    void (*_onReceive)(int);
    void (*_onTxDone)();
    uint8_t _offset = 0;
    uint32_t _data;


};