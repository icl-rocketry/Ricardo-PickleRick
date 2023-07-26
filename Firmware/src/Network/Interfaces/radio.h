#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "Storage/systemstatus.h"

#include <libriccore/riccorelogging.h>

#include <memory>
#include <vector>
#include <string>
#include <queue>

#include <LoRa.h>

#include <librnp/rnp_interface.h>
#include <librnp/rnp_packet.h>

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
    float snr;
    long freqError;
};

struct RadioConfig
{
    long frequency;
    uint8_t sync_byte;
    long bandwidth;
    int spreading_factor;
};

class Radio : public RnpInterface
{
public:
    Radio(SPIClass &spi, SystemStatus &systemstatus, RADIO_MODE mode = RADIO_MODE::SIMPLE, int cs,int reset, int dio, uint8_t id = 2, std::string name = "Radio");
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
    SystemStatus &_systemstatus; // pointer to system status object

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

    static constexpr RadioConfig defaultConfig{868e6,0xF3,500e3,7};
};
