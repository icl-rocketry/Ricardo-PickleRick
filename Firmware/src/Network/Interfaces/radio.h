#pragma once
//c++ stl
#include <memory>
#include <vector>
#include <array>
#include <string>
#include <queue>
#include <atomic>
//arduino + esp32 
#include <Arduino.h>
#include <SPI.h>
//freertos
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

//Ric libraries
#include <libriccore/riccorelogging.h>

#include <librnp/rnp_interface.h>
#include <librnp/rnp_packet.h>
#include <Preferences.h>

// /lib
#include <LoRa.h>
 
#include <Config/types.h>
#include <Config/systemflags_config.h>


enum class RADIO_MODE : uint8_t
{
    SIMPLE,
    TURN_TIMEOUT,
    SYNC
};

struct RadioInterfaceInfo : public RnpInterfaceInfo
{
    size_t sendBufferSize;
    bool sendBufferOverflow;

    RADIO_MODE mode;
    uint32_t prevTimeSent;
    uint32_t prevTimeReceived;

    int rssi;
    int packet_rssi;
    float snr;
    float packet_snr;
    long freqError;
};

struct RadioConfig
{
    long frequency; //in Hz
    uint8_t sync_byte;
    long bandwidth; //in Hz
    int spreading_factor; 
    int txPower; //in dB
};
class Radio : public RnpInterface
{
public:
    Radio(SPIClass &spi, int cs,int reset, int dio, Types::CoreTypes::SystemStatus_t &systemstatus, RADIO_MODE mode = RADIO_MODE::SIMPLE,  uint8_t id = 2, std::string name = "Radio");
    ~Radio();

    void setup() override;

    void sendPacket(RnpPacket &data) override;
    void update() override;

    const RnpInterfaceInfo *getInfo() override;
    const RadioConfig& getConfig();
    void setConfig(RadioConfig config);
    void setConfig(RadioConfig config,bool overrideNVS);
    void restart();

    //!TEMPORARY - NEEDS TO BE REWRITTEN TO BE CONSISTENT WITH THE REST OF THE CALIBARTION BACKEND
    void setFreq(long freq){
        _config.frequency = freq;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Lora freq updated");
        saveConf();
    };
    void setPower(uint8_t power){
        _config.txPower = power;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Lora power updated");
        saveConf();
    };

    void setSF(uint8_t SF){
        _config.spreading_factor = SF;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Lora SF updated");
        saveConf();
    };

    void setBW(long bandwidth){
        _config.bandwidth = bandwidth;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Lora BW updated");
        saveConf();
    };
    void setSW(uint8_t SyncWord){
        _config.sync_byte = SyncWord;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Lora SW updated");
        saveConf();
    };

    void saveConf(){

        Preferences NVS;
        NVS.begin("RadioConfig");
        NVS.putLong("Freq",_config.frequency);
        NVS.putLong("BW",_config.bandwidth);
        NVS.putUShort("SF",_config.spreading_factor);
        NVS.putInt("SW",_config.sync_byte);
        NVS.putUShort("Pwr",_config.txPower);
        NVS.end();
    };

    void loadConf(){

        Preferences NVS;
        _config = defaultConfig;
        if(!NVS.begin("RadioConfig")){
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Radio config not found, reverting to default!");
            return;
        };

        long freq = NVS.getLong("Freq");
        if(!freq){
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("LoRa Frequency not configured!");
        }
        else{
            _config.frequency = freq;
        }

        long BW = NVS.getLong("BW");
        if(!BW){
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("LoRa BW not configured!");
        }
        else{
            _config.bandwidth = BW;
        }
        
        uint8_t SF = NVS.getUShort("SF");
        if(!SF){
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("LoRa SF not configured!");
        }
        else{
            _config.spreading_factor = SF;
        } 

        uint8_t PWR = NVS.getUShort("Pwr");
        if(!PWR){
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("LoRa power not configured!");
        }
        else{
            _config.txPower = PWR;
        }

        int16_t SW = NVS.getInt("SW",-1);
        if(SW == -1){
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("LoRa sync byte not configured!");
        }
        else{
            _config.sync_byte = SW;
        }        

        NVS.end();
    };

private:
    LoRaClass loraRadio;

    RadioConfig _config;

    SPIClass &_spi;              // pointer to spi class
    Types::CoreTypes::SystemStatus_t &_systemstatus; // pointer to system status object

    const int csPin;
    const int resetPin;
    const int dioPin;


    RadioInterfaceInfo _info;

    //send buffer
    std::queue<std::vector<uint8_t>> _sendBuffer;
    size_t _currentSendBufferSize;

    //receive buffer
    QueueHandle_t m_receiveBuffer;
    static constexpr size_t m_receiveBufferSize = 20; //max size is 5120B
    static constexpr size_t m_receiveBufferElementSize = sizeof(std::vector<uint8_t>*);

    std::array<uint8_t,m_receiveBufferSize*m_receiveBufferElementSize> m_receiveBufferStorage;
    StaticQueue_t m_receiveBufferMgmt;

    //receive buffer error handling
    uint8_t m_prevDumpedPackets;
    std::atomic<uint8_t> m_dumpedPackets;

    std::atomic<bool> m_txDone;

    void serviceReceiveBuffer();
    void serviceSendBuffer();

    size_t send(std::vector<uint8_t> &data);
    // void checkTx();
    void sendFromBuffer();

    //Interrupt Handlers
    //TODO handle class destruction safely handling dandling this reference!
    IRAM_ATTR void txDoneHandler();
    IRAM_ATTR void rxHandler(int packetSize);

    static constexpr int turnTimeout = 250;
    bool _received;

    static constexpr RadioConfig defaultConfig{static_cast<long>(868E6),0xF3,static_cast<long>(500E3),7,20};

    enum class SYNCMODE_STATE:uint8_t
    {
        DISCONNECTED,
        CONNECTED
    };
    enum class SYNCMODE_MODE:uint8_t
    {
        RX,
        TX
    };
    
    static constexpr uint8_t syncPacketStartByte = 0xBF;

    struct SyncModeInfo{
        uint32_t guardTime; // in ms
        uint32_t beaconDelta; // in ms
        uint32_t connectionTimeout;
        SYNCMODE_STATE state;
        SYNCMODE_MODE mode;
        

    };
    //some defaults
    SyncModeInfo syncmodeinfo{500,1000,10000,SYNCMODE_STATE::DISCONNECTED,SYNCMODE_MODE::RX};

    void syncModeTransmit_Hook();
    void syncModeReceive_Hook(std::unique_ptr<RnpPacketSerialized> packet_ptr);

    

};
