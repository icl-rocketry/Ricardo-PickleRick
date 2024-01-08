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
#include <librnp/rnp_networkmanager.h>

// /lib
#include <LoRa.h>

#include <Config/types.h>
#include <Config/systemflags_config.h>


struct RadioConfig
{
    double frequency;
    uint8_t sync_byte;
    double bandwidth;
    int spreading_factor;
    int cr_denominator;
    int preamble_length;
    uint8_t gain;

    static constexpr uint8_t max_payload_length = 255;
    static constexpr uint8_t max_ack_length = 4;        // tdma header size
};

enum TDMA_MODE : uint8_t
{
    DISCOVERY,
    TRANSMIT,
    RECEIVE
};

struct RadioInterfaceInfo : public RnpInterfaceInfo
{
    size_t sendBufferSize;
    bool sendBufferOverflow;

    TDMA_MODE mode;
    uint32_t prevTimeSent;

    int rssi;
    float snr;
    long freqError;
};

enum PacketType : uint8_t
{
    NORMAL,     // rnp packet
    ACK,
    NACK,
    JOINREQUEST,
    HEARTBEAT
};

class TDMA : public RnpInterface
{
public:

    TDMA(SPIClass &spi, int cs, int reset, int dio, Types::CoreTypes::SystemStatus_t &systemstatus, 
            RnpNetworkManager& networkmanager, uint8_t id = 2, std::string name = "TDMA Radio");

    void setup() override;
    void setConfig(RadioConfig config);
    const RadioConfig& getConfig();
    const RnpInterfaceInfo* getInfo();
    void sendPacket(RnpPacket &data) override;
    void update();

private:
    RadioConfig _config;
    SPIClass &_spi;
    Types::CoreTypes::SystemStatus_t& _systemstatus;
    const int _csPin;
    const int _resetPin;
    const int _dioPin;
    size_t _currentSendBufferSize;
    bool _txDone;
    bool _received = false;
    RnpNetworkManager &_networkmanager;

    // LOW LEVEL DRIVER STUFF

    LoRaClass loraRadio;
    void radioSetup();
    void radioRestart();
    void getPacket();
    void checkSendBuffer();
    size_t send(std::vector<uint8_t> &data);
    void checkTx();
    void sendFromBuffer();
    size_t bytes_written;


    // TOP LEVEL TDMA DRIVER
    
    void discovery();
    void rx();
    void tx();
    TDMA_MODE currentMode = TDMA_MODE::DISCOVERY;    

    void calcTimewindowDuration();
    double calcPacketToF(int Lpayload);

    void initNetwork();
    void generateTDMAHeader(PacketType packettype);
    void generateTDMAHeader(PacketType packettype, uint8_t info);
    void sync();

    std::queue<std::vector<uint8_t>> _sendBuffer; 

    static constexpr double discoveryTimeout = 60e6;     //TODO: this needs a proper calc once channel hopping is implemented
    double timeEnteredDiscovery;
    double timeJoinRequestSent;        

    double timewindowDuration;
    uint8_t timewindows;
    uint8_t txTimewindow;
    uint8_t currTimewindow;
    double timeMovedTimewindow = 0;

    uint8_t countsNoTx;
    uint8_t maxCountsNoTx = 10;

    uint8_t countsNoAck;
    uint8_t maxCountsNoAck = 2;  // only 2 resends

    std::vector<uint8_t> registeredNodes;   // list of rnp nodes on network

    PacketType receivedPacketType;
    double timePacketReceived;    
    uint8_t packetSource;
    uint8_t packetDest;
    uint8_t packetRegNodes;
    uint8_t packetTimewindow;
    uint8_t packetInfo;
    int packetSize;

    bool enteredDiscovery = false;
    bool joinRequestSent = false;
    bool txWindowDone = false;
    bool rxWindowDone = false;
    bool packetSent = false;
    bool synced = false;
    bool acked = false;
    bool packetForMe;

    std::vector<uint8_t> TDMAHeader;


    RadioConfig defaultConfig{(long)868E6, 0x12, (long)250E3, 7, 5, 8, 6};
    RadioInterfaceInfo _info;
};