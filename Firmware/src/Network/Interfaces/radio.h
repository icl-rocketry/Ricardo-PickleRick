#pragma once
//c++ stl
#include <memory>
#include <vector>
#include <string>
#include <queue>
//arduino + esp32 
#include <Arduino.h>
#include <SPI.h>

//Ric libraries
#include <libriccore/riccorelogging.h>

#include <librnp/rnp_interface.h>
#include <librnp/rnp_packet.h>

// /lib
#include <LoRa.h>
 
#include <Config/types.h>
#include <Config/systemflags_config.h>


enum class RADIO_MODE : uint8_t
{
    SIMPLE,
    TURN_TIMEOUT
};

struct RadioInterfaceInfo : public RnpInterfaceInfo
{
    size_t sendBufferSize;
    bool sendBufferOverflow;

    RADIO_MODE mode;
    uint32_t prevTimeSent;

    int rssi;
    int packet_rssi;
    float snr;
    float packet_snr;
    long freqError;
};

struct RadioConfig
{
    long frequency;
    uint8_t sync_byte;
    long bandwidth;
    int spreading_factor;
    int txPower;
};

class Radio : public RnpInterface
{
public:
    Radio(SPIClass &spi, int cs,int reset, int dio, Types::CoreTypes::SystemStatus_t &systemstatus, RADIO_MODE mode = RADIO_MODE::SIMPLE,  uint8_t id = 2, std::string name = "Radio");
    void setup() override;

    void sendPacket(RnpPacket &data) override;
    void update() override;
    const RnpInterfaceInfo *getInfo() override;
    const RadioConfig& getConfig();
    void setConfig(RadioConfig config);
    void restart();

private:
    LoRaClass loraRadio;

    RadioConfig _config;

    SPIClass &_spi;              // pointer to spi class
    Types::CoreTypes::SystemStatus_t &_systemstatus; // pointer to system status object

    const int csPin;
    const int resetPin;
    const int dioPin;


    RadioInterfaceInfo _info;

    std::queue<std::vector<uint8_t>> _sendBuffer;
    size_t _currentSendBufferSize;

    bool _txDone;

    void getPacket();
    void checkSendBuffer();
    size_t send(std::vector<uint8_t> &data);
    void checkTx();
    void sendFromBuffer();

    static constexpr int turnTimeout = 250;
    bool _received;

    static constexpr RadioConfig defaultConfig{static_cast<long>(868E6),0xF3,static_cast<long>(500E3),7,20};
};
