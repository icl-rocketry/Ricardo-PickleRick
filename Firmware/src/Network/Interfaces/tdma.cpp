#include "tdma.h"
#include <libriccore/riccorelogging.h>


TDMA::TDMA(SPIClass& spi, int cs,int reset, int dio,Types::CoreTypes::SystemStatus_t& systemstatus, RnpNetworkManager& networkmanager, uint8_t id, std::string name):
RnpInterface(id,name),
_config(defaultConfig),
_spi(spi),
_systemstatus(systemstatus),
_csPin(cs),
_resetPin(reset),
_dioPin(dio),
_currentSendBufferSize(0),
_txDone(true),
_received(false),
_networkmanager(networkmanager)
{
    _info.MTU = 256;
    _info.sendBufferSize = 2048;
    _info.mode=currentMode;
};

void TDMA::setup(){
    radioSetup();

    if(!_systemstatus.flagSetOr(SYSTEM_FLAG::ERROR_LORA)){
        calcTimewindowDuration();
        registeredNodes.push_back(_networkmanager.getAddress());    // add own address to list
        timewindows = registeredNodes.size() + 1;                   // n+1 timewindows
    }

    timeMovedTimewindow = micros();
};



void TDMA::update(){

    if (micros() - timeMovedTimewindow >= timewindowDuration){
        currTimewindow = (currTimewindow + 1) % timewindows;
        //RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(std::to_string(currTimewindow));
        timeMovedTimewindow = micros();
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(std::to_string(timeMovedTimewindow));

        packetSent = false;
        txWindowDone = false;
        rxWindowDone = false;
    }


    if (currentMode == TDMA_MODE::DISCOVERY){
        discovery();
    }
    else{   

        if(currTimewindow == txTimewindow){
            currentMode = TDMA_MODE::TRANSMIT;
            if(!txWindowDone){
                tx();
            }
        }
        else{
            currentMode = TDMA_MODE::RECEIVE;
            if(!rxWindowDone){
                rx();
            }
        }
    }
};


void TDMA::discovery(){

    std::vector<uint8_t> joinRequest(tdmaHeaderSize);   // variable declaration apparently needs to be outside switch case :(

    switch(currentDiscoveryPhase) {


        case DISCOVERY_PHASE::ENTRY: {
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered Discovery");
            timeEnteredDiscovery = micros();                    // timestamp entry
            currentDiscoveryPhase = DISCOVERY_PHASE::SNIFFING;  // transition to next phase
            break;
        }

        case DISCOVERY_PHASE::SNIFFING: {
            getPacket();        // scan for packets

            if (micros() - timeEnteredDiscovery > discoveryTimeout){
                currentDiscoveryPhase = DISCOVERY_PHASE::INIT_NETWORK;  // transition to network initialisation
            }

            if (_received){
                RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Network detected");
                currentDiscoveryPhase = DISCOVERY_PHASE::SYNCING;       // transition to network syncing
            }
            break;
        }
        

        case DISCOVERY_PHASE::INIT_NETWORK: {
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Initialising network");
            initNetwork();
            currentDiscoveryPhase = DISCOVERY_PHASE::EXIT;              // transition to exit phase
            break;
        }


        case DISCOVERY_PHASE::SYNCING: {
            sync();
            currentDiscoveryPhase = DISCOVERY_PHASE::JOIN_REQUEST;      // transition to requesting to join
            break;
        }


        case DISCOVERY_PHASE::JOIN_REQUEST: {
            if(currTimewindow == txTimewindow){
                generateTDMAHeader(joinRequest, PacketType::JOINREQUEST, packetSource);
                if(send(joinRequest) > 0){                                      // bytes written successfully
                    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Join request sent");
                    timeJoinRequestSent = micros();
                    currentDiscoveryPhase = DISCOVERY_PHASE::JOIN_REQUEST_RESPONSE; // transition to waiting for response
                }
            } 
            break;
        }


        case DISCOVERY_PHASE::JOIN_REQUEST_RESPONSE: {

            getPacket();        // scan for response 

            if(_received && receivedPacketType == PacketType::ACK && packetDest == _networkmanager.getAddress()){
                RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Joined network");
                registeredNodes.push_back(_networkmanager.getAddress()); // add self to list
                registeredNodes.push_back(packetDest);       // maybe should not add to the registered nodes list
                timewindows = packetRegNodes + 1;   // set local number of timewindows to match network
                currentDiscoveryPhase = DISCOVERY_PHASE::EXIT;
            }

            else if(_received && receivedPacketType == PacketType::NACK){  
                RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Node joined before");
                txTimewindow = packetInfo;
                registeredNodes.push_back(_networkmanager.getAddress()); // add self to list
                registeredNodes.push_back(packetDest);       // maybe should not add to the registered nodes list
                timewindows = packetRegNodes + 1;
                currentDiscoveryPhase = DISCOVERY_PHASE::EXIT;
            }

            if (micros() - timeJoinRequestSent > joinRequestExpiryTime){       // request expired
                currentDiscoveryPhase = DISCOVERY_PHASE::JOIN_REQUEST;  // try again
            }
            break;
        }

        
        case DISCOVERY_PHASE::EXIT: {
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Exiting discovery");
            currentMode = TDMA_MODE::TRANSMIT;      // to exit out of discovery, assign any other mode other than discovery
            break;
        }


        default: {
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("ERROR: Unknown discovery phase");
            break;
        }

    }

}


void TDMA::sync(){
    // back timestamping when the node was meant to move windows
    timeMovedTimewindow = timePacketReceived - static_cast<uint64_t>(calcPacketToF(packetSize)*1e6f);
    
    currTimewindow = packetTimewindow;    // sync local current timewindow to network
    txTimewindow = packetRegNodes;        // send in the n+1th timewindow
    timewindows = packetRegNodes+1;       // update local number of timewindows
    synced = true;                        // syncing complete
}



void TDMA::initNetwork(){
    timewindows = registeredNodes.size() + 1;    //n+1 timewindows
    txTimewindow = 0;  //of this node
    currTimewindow = txTimewindow;
    timeMovedTimewindow = micros();
}

void TDMA::generateTDMAHeader(std::vector<uint8_t> &TDMAHeader, PacketType packettype, uint8_t destinationNode){
    TDMAHeader = {static_cast<uint8_t>(packettype), static_cast<uint8_t>(registeredNodes.size()), 
                    currTimewindow, static_cast<uint8_t>(_networkmanager.getAddress()), 
                    static_cast<uint8_t>(destinationNode), static_cast<uint8_t>(255)};
}

void TDMA::generateTDMAHeader(std::vector<uint8_t> &TDMAHeader, PacketType packettype, uint8_t destinationNode, uint8_t info){
    TDMAHeader = {static_cast<uint8_t>(packettype), static_cast<uint8_t>(registeredNodes.size()), 
                    currTimewindow, static_cast<uint8_t>(_networkmanager.getAddress()), 
                    static_cast<uint8_t>(destinationNode), info};
}

void TDMA::unpackTDMAHeader(std::vector<uint8_t> &packet){
    uint8_t initial_size = packet.size();

    receivedPacketType = static_cast<PacketType>(packet.front());
    packet.erase(packet.begin());

    packetRegNodes = packet.front();
    packet.erase(packet.begin());

    packetTimewindow = packet.front();
    packet.erase(packet.begin());

    packetSource = packet.front();
    packet.erase(packet.begin());

    packetDest = packet.front();
    packet.erase(packet.begin());

    if(packet.front() != 255){
        packetInfo = packet.front();
    }
    packet.erase(packet.begin());

    if (initial_size - static_cast<uint8_t>(packet.size()) != tdmaHeaderSize){
        throw std::runtime_error("Error unpacking TDMA header");
    }

}

void TDMA::tx(){

    if(_sendBuffer.size() > 0){    //buffer not empty

        if(countsNoAck > maxCountsNoAck){  // check if exceed limit on resends
            uint8_t popped_packet_size = sizeof(_sendBuffer.back());
            _sendBuffer.pop();      // pop old packet
            _currentSendBufferSize -= popped_packet_size;
            countsNoAck = 0;      // reset counter#
            txWindowDone = true;
        }
        if(!packetSent){
            bytes_written = send(_sendBuffer.front());  // send from front of buffer
            if (bytes_written){
                RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("RNP packet sent");
                countsNoAck ++;  // just trust me bro, it makes sense
                countsNoTx = 0; 
            }
        }
        else{                   // packet has been sent
            getPacket();        // scan for acks
            if (_received && receivedPacketType == PacketType::ACK){ // packet got acked
                uint8_t popped_packet_size = sizeof(_sendBuffer.back());
                _sendBuffer.pop();      // remove packet from send queue
                _currentSendBufferSize -= popped_packet_size;
                countsNoAck = 0;      // reset counter
                txWindowDone = true;   // exit tx mode
            }
        } 

    }
    else{                           // buffer empty

        if (countsNoTx >= maxCountsNoTx){        // node didn't transmit in a long time
            //RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("transmit heartbeat");
            std::vector<uint8_t> heartbeatPacket(tdmaHeaderSize);
            generateTDMAHeader(heartbeatPacket, PacketType::HEARTBEAT, 0);
            send(heartbeatPacket);  // send skipping buffer
            countsNoTx = 0;
            txWindowDone = true;
        }
        else{ 
            countsNoTx ++;             // update counter
            txWindowDone = true;       // exit tx mode
        }

    }

}

void TDMA::rx(){

    getPacket();    // scan for packets

    if(_received){

        //timeMovedTimewindow = timePacketReceived - static_cast<uint64_t>(calcPacketToF(packetSize)*1e6f);   //resyncing
        std::vector<uint8_t> ackPacket(tdmaHeaderSize);     // initialising ack packet

        switch (receivedPacketType) {

            case PacketType::JOINREQUEST: {                             // handling join request
                
                RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Received join request");

                auto it = find(registeredNodes.begin(), registeredNodes.end(), packetSource);

                if(it == registeredNodes.end()){                        // node has not been registered yet
                    registeredNodes.push_back(packetSource);            // add to node list
                    timewindows = registeredNodes.size()+1;             // update number of timewindows
     
                    generateTDMAHeader(ackPacket, PacketType::ACK, packetSource);     //ack join request
                    send(ackPacket);            
                    rxWindowDone = true;
                }
                else{                                                   // node has already registered
                    uint8_t requesterTxTimewindow = static_cast<uint8_t>(it-registeredNodes.begin());
                    generateTDMAHeader(ackPacket, PacketType::NACK, packetSource, requesterTxTimewindow); // nack join request
                    send(ackPacket);            
                    rxWindowDone = true;               
                }
                break;
            }
                
            
            case PacketType::NORMAL: {                                  // handling RNP packet

                generateTDMAHeader(ackPacket, PacketType::ACK, packetSource);
                send(ackPacket);
                rxWindowDone = true; 
                break; 
            }

            case PacketType::HEARTBEAT: {                               // handling heartbeat packet
                break;
            }

            default: {                                                  // handling other packet
                RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Received unexpected packet type: " + std::to_string(receivedPacketType));
                break;
            }

        }
    }
}


void TDMA::calcTimewindowDuration(){
    float maxTframe = 2;                 //assuming 2 seconds
    float clock_drift = 2E-5;            //us/s
    float Tg = maxTframe*clock_drift;    //guard time   
    float ff = 1.1;                        //magic fudge factor
    timewindowDuration = ff*(calcPacketToF(_config.max_payload_length) + calcPacketToF(_config.max_ack_length) + Tg)*1e6f;
}

float TDMA::calcPacketToF(int Lpayload){
    float Rs = _config.bandwidth/pow(2,_config.spreading_factor);
    float CR = _config.cr_denominator - 4;
    float Tsym = 1/Rs;
    float Tpreamble = (_config.preamble_length + 4.25)*Tsym;
    float Tpayload = Tsym*(8 + ceil((8*Lpayload - 4*_config.spreading_factor + 44)/(4*_config.spreading_factor))*(CR + 4));
    return Tpreamble + Tpayload;
}


void TDMA::sendPacket(RnpPacket& data)
{
    const size_t dataSize = data.header.size() + data.header.packet_len;
    if (dataSize > _info.MTU){ // will implement packet segmentation here at a later data
        //RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Packet Exceeds Radio MTU");
        ++_info.txerror;
        return;
    }
    if (dataSize + _currentSendBufferSize > _info.sendBufferSize){
        _systemstatus.newFlag(SYSTEM_FLAG::ERROR_LORA," Lora Send Buffer Overflow!");
        ++_info.txerror;
        _info.sendBufferOverflow = true;
        return;
    }

    std::unique_ptr<std::vector<uint8_t>> serializedPacket = std::make_unique<std::vector<uint8_t>>();
    data.serialize(*serializedPacket);
    std::vector<uint8_t> tdmaHeader(tdmaHeaderSize);
    generateTDMAHeader(tdmaHeader, PacketType::NORMAL, 0);  //TODO:: fix this
    serializedPacket->insert(serializedPacket->begin(), tdmaHeader.begin(), tdmaHeader.end());
    _sendBuffer.push(*serializedPacket);
    _info.sendBufferOverflow = false;
    _currentSendBufferSize += dataSize;

}



//LOW LEVEL SHIT below

void TDMA::radioSetup(){
    //setup loraRadio module
    loraRadio.setPins(_csPin,_resetPin,_dioPin);
    loraRadio.setSPI(_spi);
    //load defaut config
    setConfig(defaultConfig);
};

void TDMA::radioRestart(){
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
    loraRadio.setCodingRate4(_config.cr_denominator);
    loraRadio.setPreambleLength(_config.preamble_length);
    loraRadio.setGain(_config.gain);
    loraRadio.enableCrc();
}

void TDMA::getPacket(){
    //TODO: add counter for time havent heard back from node
    //check if radio is still transmitting
    if (loraRadio.isTransmitting()){
        return;
    }

    int size = loraRadio.parsePacket();

    if (size>0){
        packetSize = size;
        _received=true; 

        timePacketReceived = micros();
        std::vector<uint8_t> data(packetSize);
        loraRadio.readBytes(data.data(),packetSize);

        //unpacking TDMA header
        try{
            unpackTDMAHeader(data);
        }
        catch (std::exception& e)
        {
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("TDMA Error: " + std::string(e.what()));
            return;
        }

        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(std::to_string(receivedPacketType));


        // TODO: fix this shit
        // if (currentMode != TDMA_MODE::DISCOVERY && packetRegNodes - static_cast<uint8_t>(registeredNodes.size()) > 0){  //local node list is shorter
        //     registeredNodes.resize(packetRegNodes);
        // }        

        if (data.size() > 0){     // packet still has smthing left in it

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

            // useful rnp header vals
            packetSource = packet_ptr->header.source;
            packetDest = packet_ptr->header.destination;

            if (packetDest == _networkmanager.getAddress()){
                packetForMe = true;
            }
            else{
                packetForMe = false;
            }

            //update source interface
            packet_ptr->header.src_iface = getID();
            _packetBuffer->push(std::move(packet_ptr));//add packet ptr  to buffer
        }
    }
}


size_t TDMA::send(std::vector<uint8_t> &data){
    if (loraRadio.beginPacket()){
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Radio Send");
        loraRadio.write(data.data(), data.size());
        loraRadio.endPacket(false); // asynchronous send 
        //_txDone = false;
        //_info.prevTimeSent = millis();
        packetSent = true;
        _received = false;
        acked = false;
        if (static_cast<PacketType>(data.front()) == PacketType::NORMAL){
            //RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Radio RNP packet Send");
        }
        return data.size();
    }else{
        return 0;
    }
}


const RnpInterfaceInfo* TDMA::getInfo()
{
     _info.rssi = loraRadio.rssi();
     _info.snr = loraRadio.packetSnr();  
     _info.freqError = loraRadio.packetFrequencyError();
     return &_info;
};

const RadioConfig& TDMA::getConfig(){return _config;};

void TDMA::setConfig(RadioConfig config)
{
    _config = config;
    radioRestart();
}