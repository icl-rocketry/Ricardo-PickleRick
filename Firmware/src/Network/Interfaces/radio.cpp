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
_txDone(true),
_received(true)
{
    _info.MTU = 256;
    _info.sendBufferSize = 2048;
    _info.mode=mode;
    _info.prevTimeSent = 0;
    _info.prevTimeReceived = 0;
};


void Radio::setup(){
    //setup loraRadio module
    loraRadio.setPins(csPin,resetPin,dioPin);
    loraRadio.setSPI(_spi);
    //load defaut config and restart the radio
    // loadConf();
    restart();
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
    checkSendBuffer(); // see if we can send 
    

}

void Radio::update(){
    getPacket();
    checkSendBuffer();
    checkTx();
    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(std::to_string(loraRadio.readRegister(0x1f)));
}


void Radio::getPacket(){
    // check if radio is still transmitting
    if (!_txDone){  // this maybe able to be replaced wiht the begin packet method
        return;
    }

    int packetSize = loraRadio.parsePacket(); // put radio back into single receive mode and check for packets

    if (packetSize){

        std::vector<uint8_t> data(packetSize);
        loraRadio.readBytes(data.data(),packetSize);
        // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Radio receive");
        // std::string message = "Packet RSSI: " + std::to_string(loraRadio.packetRssi()) + ", SNR: " + std::to_string(loraRadio.packetSnr());
        // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(message);
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

        //place radio back into rx mode
        loraRadio.parsePacket();
        
        

    }
}

void Radio::checkSendBuffer(){
    // if (_sendBuffer.size() == 0){
    //     return; // exit if nothing in the buffer
    // }
    
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
        _txDone = false;
        _info.prevTimeSent = millis();
        _received = false; // used for turn_timeout mode
        return data.size();
    }else{
        return 0;
    }
}

void Radio::checkTx(){
    if (_txDone){
        return;
    }
    if (!loraRadio.isTransmitting()){
        _txDone = true;
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
        loraRadio.parsePacket();
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