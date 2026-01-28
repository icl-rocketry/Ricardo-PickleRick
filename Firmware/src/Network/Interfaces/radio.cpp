#include "radio.h"


#include <libriccore/riccorelogging.h>

#include <SPI.h>

#include <LoRa.h>


#include <Config/types.h>
#include <Config/systemflags_config.h>

#include <memory>
#include <vector>

#include <librnp/rnp_interface.h>
#include <librnp/rnp_packet.h>

#include "Config/pinmap_config.h"

Radio::Radio(SPIClass& spi, int cs,int reset, int dio,Types::CoreTypes::SystemStatus_t& systemstatus, RADIO_MODE mode,uint8_t id,std::string name):
RnpInterface(id,name),
_config(defaultConfig),
_spi(spi),
_systemstatus(systemstatus),
csPin(cs),
resetPin(reset),
dioPin(dio),
_currentSendBufferSize(0),
m_receiveBuffer( xQueueCreateStatic(m_receiveBufferSize,
                                    m_receiveBufferElementSize,
                                    m_receiveBufferStorage.data(),
                                    &m_receiveBufferMgmt) ),
m_prevDumpedPackets(0),
m_dumpedPackets(0),
m_txDone(true),
_received(true)
{
    _info.MTU = 256;
    _info.sendBufferSize = 2048;
    _info.mode=mode;
    _info.prevTimeSent = 0;
    _info.prevTimeReceived = 0;
};

Radio::~Radio()
{
    //temporary cleanup of interrupt handlers idk if it will actually cover all cases
    //! check destruction order!!
    loraRadio.onReceive([](int packetSize){return;});
    loraRadio.onTxDone([](){return;});
}

void Radio::setup(){
    //setup loraRadio module
    loraRadio.setPins(csPin,resetPin,dioPin);
    loraRadio.setSPI(_spi);
    //setup intterupt handlers
    loraRadio.onReceive([this](int packetSize){this->rxHandler(packetSize);});
    loraRadio.onTxDone([this](){this->txDoneHandler();});
    //load defaut config and restart the radio
    // loadConf();
    restart();
    loraRadio.setTxPower(20,1); //20 dbm, mux antenna output to PA_Boost (req. for 20dbm)
};



void Radio::sendPacket(RnpPacket& data)
{
    const size_t dataSize = data.header.size() + data.header.packet_len;
    if (dataSize > _info.MTU){ // will implement packet segmentation here at a later data
         RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Packet Exceeds Radio MTU");
        ++_info.txerror;
        return;
    }
    if (dataSize + _currentSendBufferSize > _info.sendBufferSize){
        _systemstatus.newFlag(SYSTEM_FLAG::ERROR_LORA,"Lora Send Buffer Overflow!");
        ++_info.txerror;
        _info.sendBufferOverflow = true;
        return;
    }

    std::vector<uint8_t> serializedPacket;
    data.serialize(serializedPacket);
    _sendBuffer.push(serializedPacket); // add to send buffer
    _info.sendBufferOverflow = false;
    _currentSendBufferSize += dataSize;
    serviceSendBuffer(); // see if we can send 
    

}


void Radio::update(){
    serviceReceiveBuffer();
    serviceSendBuffer();

  

}


void Radio::serviceReceiveBuffer(){

    if (m_receiveBuffer == nullptr){
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Radio Receive Buffer Queue Not Initialized!");
        return;
    }

    if (!uxQueueMessagesWaiting(m_receiveBuffer)){
        return;
    }

    std::vector<uint8_t> data;

    std::vector<uint8_t>* dataPtr;

    if (!xQueueReceive(m_receiveBuffer, static_cast<void*>(&dataPtr), 0)){
        //log the error
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Radio Receive Buffer Queue Error");
        return;
    }

    //copy data out of pointer
    data.swap(*dataPtr);
    delete dataPtr;

    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Radio receive");

    if (_packetBuffer == nullptr){
        return;
    }
    std::unique_ptr<RnpPacketSerialized> packet_ptr;

    try
    {
        packet_ptr = std::make_unique<RnpPacketSerialized>(data);
    }
    catch (std::exception& e)
    {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Deserialization error: " + std::string(e.what()));
        return;
    }

    //probably want to eventualyl move this to the isr for more accurate reception
    _received=true;
    _info.prevTimeReceived = millis();

    //update source interface
    packet_ptr->header.src_iface = getID();

    switch(_info.mode)
    {
        case(RADIO_MODE::SYNC):
        {
            syncModeReceive_Hook(std::move(packet_ptr));
            break;
        }
        default:
        {
            _packetBuffer->push(std::move(packet_ptr));//add packet ptr  to buffer
            break;
        }
    }    

}

void Radio::serviceSendBuffer(){


    if (!m_txDone)
    {
        return;
    }
    
    switch(_info.mode)
    {
        case(RADIO_MODE::SIMPLE):
        {  
            if (_sendBuffer.size())
            {
                sendFromBuffer();
            }
            
            break;
        }
        case(RADIO_MODE::TURN_TIMEOUT):
        {
            if (_sendBuffer.size()){
                if (_received || (millis()-_info.prevTimeSent > turnTimeout)){
                    sendFromBuffer();
                }
            }
            break;
        }
        case(RADIO_MODE::SYNC):
        {
            syncModeTransmit_Hook();
            break;
        }
        default:
        {
            break;
        }
    }
   
}

void Radio::sendFromBuffer()
{
     // check if radio is busy, if it isnt then send next packet
    size_t bytes_written = send(_sendBuffer.front());
    if (bytes_written){ // if we succesfully send packet
        _sendBuffer.pop(); //remove packet from buffer
        _currentSendBufferSize -= bytes_written;
    }

}

size_t Radio::send(std::vector<uint8_t> &data){
    if (loraRadio.beginPacket()){
        // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Radio Send");
        loraRadio.write(data.data(), data.size());
        loraRadio.endPacket(true); // asynchronous send 
        m_txDone = false;
        _info.prevTimeSent = millis();
        _received = false; // used for turn_timeout mode
        return data.size();
    }else{
        return 0;
    }
}

IRAM_ATTR void Radio::txDoneHandler()
{
    m_txDone = true;
    loraRadio.receive();
}

IRAM_ATTR void Radio::rxHandler(int packetSize)
{
    if (!packetSize){
        // idk problem for future me to sort out
        return;
    }
    //copy packet out
    std::vector<uint8_t> radioData(packetSize);
    loraRadio.readBytes(radioData.data(),packetSize);

    if (m_receiveBuffer == nullptr){
        //queue failed to intialize
        //TODO log this
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Radio Receive Buffer Queue Not Initialized!");
        return;
    }
    
    if (!uxQueueSpacesAvailable(m_receiveBuffer)){
        //TODO handle overflow
        m_dumpedPackets++;
        uint8_t nDumped = m_dumpedPackets - m_prevDumpedPackets;
        m_prevDumpedPackets = m_dumpedPackets;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Radio Receive Buffer Overflow: " + std::to_string(nDumped) + " packets dumped");
 
    }
    else {
        std::vector<uint8_t>* radioDataPtr = new std::vector<uint8_t>(packetSize);
        radioData.swap(*radioDataPtr);

        if(!xQueueSendToBack(m_receiveBuffer, static_cast<void*>(&radioDataPtr), 0)){
            delete radioDataPtr;
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Failed to place packet in receive buffer");
        }

    }

}

const RnpInterfaceInfo* Radio::getInfo()
{
     _info.rssi = loraRadio.rssi();
     _info.packet_rssi = loraRadio.packetRssi();
     _info.snr = loraRadio.packetSnr();  
     _info.packet_snr = loraRadio.packetSnr();
     _info.freqError = loraRadio.packetFrequencyError();
     return &_info;
};

const RadioConfig& Radio::getConfig(){return _config;};

void Radio::setConfig(RadioConfig config)
{
    _config = config;
    restart();
}

void Radio::setConfig(RadioConfig config, bool overrideNVS)
{
    if (overrideNVS)
    {
        setConfig(config);
    }
}

void Radio::restart(){
    if (!loraRadio.begin(_config.frequency)){
        _systemstatus.newFlag(SYSTEM_FLAG::ERROR_LORA,"loraRadio Failed to start!");     
        return;
    };
    if (_systemstatus.flagSetOr(SYSTEM_FLAG::ERROR_LORA)){
        _systemstatus.deleteFlag(SYSTEM_FLAG::ERROR_LORA);
    }
    
    loraRadio.setSyncWord(_config.sync_byte);
    loraRadio.setSignalBandwidth(_config.bandwidth);
    loraRadio.setSpreadingFactor(_config.spreading_factor);
    loraRadio.enableCrc();
    loraRadio.setTxPower(_config.txPower,1);
    //place radio into receive mode
    loraRadio.receive();

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(std::string("Radio config: ") + std::string("Freq = ") + std::to_string(_config.frequency) + std::string(", BW = ") + std::to_string(_config.bandwidth) +
                    std::string(", SF = ") + std::to_string(_config.spreading_factor) + std::string(", TxPower = ") + std::to_string(_config.txPower)); 

}

void Radio::syncModeTransmit_Hook()
{
    if (syncmodeinfo.mode == SYNCMODE_MODE::RX && 
        millis() - _info.prevTimeReceived < syncmodeinfo.guardTime)
    {
        return;
    }

    if (millis() - _info.prevTimeReceived > syncmodeinfo.connectionTimeout)
    {
        syncmodeinfo.state = SYNCMODE_STATE::DISCONNECTED;
    }

    if (_sendBuffer.size())
    {
        sendFromBuffer();
    }
    else //send a sync packet
    { 
        //construct sync packet
        //! using rnp pakcet for ease here but could be a future optimization
        //create empty packet with empty header -> default values
        RnpPacket syncPacket(RnpHeader{});
        //update start byte to refelct different protocol
        syncPacket.header.start_byte = syncPacketStartByte;

        std::vector<uint8_t> serializedSyncPacket;
        syncPacket.serialize(serializedSyncPacket);
        
        switch (syncmodeinfo.state)
        {
            case (SYNCMODE_STATE::DISCONNECTED):
            {
                //in disconnected state we beacon the sync messages wrt the beacon delta 
                if ( millis() - _info.prevTimeSent > syncmodeinfo.beaconDelta)
                {
                    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Beacon Send");
                    send(serializedSyncPacket);
                }
                break;
            }
            case (SYNCMODE_STATE::CONNECTED):
            {
                if ( millis() - _info.prevTimeSent > 100)
                {
                    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Sync Send");
                    send(serializedSyncPacket);
                }
                break;
            }
        }
        //!
        // loraRadio.parsePacket();
    }
    // place driver in receive mode
    syncmodeinfo.mode = SYNCMODE_MODE::RX;
    //force driver back into rx mode?


}

void Radio::syncModeReceive_Hook(std::unique_ptr<RnpPacketSerialized> packet_ptr)
{   
    //we have received so place driver in transmit mode
    syncmodeinfo.mode = SYNCMODE_MODE::TX;
    //update state to connected
    syncmodeinfo.state = SYNCMODE_STATE::CONNECTED;

    switch(packet_ptr->header.start_byte)
    {
        case (0xAF):
        {
            //normal rnp packet
            _packetBuffer->push(std::move(packet_ptr));//add packet ptr  to buffer
            break;
        }
        case (0xBF):
        {
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Sync Receive");
            //sync packet
            //dont push sync packets to packet buffer, just dump them yea
            break;
        }


    }
   
    
}