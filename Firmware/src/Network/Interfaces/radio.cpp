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

Radio::Radio(SPIClass& spi, int cs,int reset, int dio,Types::CoreTypes::SystemStatus_t& systemstatus,RADIO_MODE mode,uint8_t id,std::string name):
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
};


void Radio::setup(){
    //setup loraRadio module
    loraRadio.setPins(csPin,resetPin,dioPin);
    loraRadio.setSPI(_spi);
    //load defaut config and restart the radio
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
        _systemstatus.newFlag(SYSTEM_FLAG::ERROR_LORA," Lora Send Buffer Overflow!");
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
        // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Radio Receive");
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

        //update source interface
        packet_ptr->header.src_iface = getID();
        _packetBuffer->push(std::move(packet_ptr));//add packet ptr  to buffer
        _received=true;

    }
}

void Radio::checkSendBuffer(){
    if (_sendBuffer.size() == 0){
        return; // exit if nothing in the buffer
    }
    
    switch(_info.mode)
    {
        case(RADIO_MODE::SIMPLE):
        {
            sendFromBuffer();
            break;
        }
        case(RADIO_MODE::TURN_TIMEOUT):
        {
            if (_received || (millis()-_info.prevTimeSent > turnTimeout)){
                sendFromBuffer();
            }
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
     _info.snr = loraRadio.packetSnr();  
     _info.freqError = loraRadio.packetFrequencyError();
     return &_info;
};

const RadioConfig& Radio::getConfig(){return _config;};

void Radio::setConfig(RadioConfig config)
{
    _config = config;
    restart();
}

void Radio::restart(){
    if (!loraRadio.begin(_config.frequency)){
        _systemstatus.newFlag(SYSTEM_FLAG::ERROR_LORA,"loraRadio setting up");      
        return;
    };
    if (_systemstatus.flagSetOr(SYSTEM_FLAG::ERROR_LORA)){
        _systemstatus.deleteFlag(SYSTEM_FLAG::ERROR_LORA);
    }
    
    loraRadio.setSyncWord(_config.sync_byte);
    loraRadio.setSignalBandwidth(_config.bandwidth);
    loraRadio.setSpreadingFactor(_config.spreading_factor);
    loraRadio.enableCrc();
    loraRadio.setTxPower(20,1);

}