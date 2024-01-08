#include "tdma.h"


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
};



void TDMA::update(){

    //timewindow shifter
    if (micros() - timeMovedTimewindow >= timewindowDuration){
        currTimewindow = (currTimewindow + 1) % timewindows;
        timeMovedTimewindow = micros();

        packetSent = false;
        txWindowDone = false;
        rxWindowDone = false;
        //txWindowDone = false;
    }


    if (currentMode == TDMA_MODE::DISCOVERY){
        discovery();
    }
    else{   // NOT in discovery
        // if (micros() - timeMovedTimewindow >= timewindowDuration){        // reached end of timewindow
        //     currTimewindow = (currTimewindow + 1) % timewindows;    // move timewindow
            
        //     // resetting important bools after changing window
        //     enteredDiscovery = false;
        //     txWindowDone = false;
        //     txWindowDone = false;
        //     packetSent = false;
        //     _received = false;

        //     timeMovedTimewindow = micros();   // timestamp beginning of new timewindow
        // }

        if(currTimewindow == txTimewindow){
            currentMode = TDMA_MODE::TRANSMIT;
            if(!txWindowDone){
                tx();
            }
        }
        else{
            currentMode = TDMA_MODE::RECEIVE;
            if(!txWindowDone){
                rx();
            }
        }
    }
};


void TDMA::discovery(){

    if (!enteredDiscovery){
        timeEnteredDiscovery = micros();
        enteredDiscovery = true;
        //Serial.println("Entered Discovery");
    }
    
    if (micros() - timeEnteredDiscovery < discoveryTimeout && !_received && !synced){
        getPacket();                // scan for packets on network
        //Serial.println("Scanning for packets");
    }
    else if (micros() - timeEnteredDiscovery > discoveryTimeout && !_received){   //no one on network
        initNetwork();
        currentMode = TDMA_MODE::TRANSMIT;  // exit discovery, can be any other mode
    }


    if (_received && !synced){     //network detected but havent synced yet
        sync();
        //Serial.println("Network detected");
    }

    if (synced && currTimewindow==txTimewindow && !joinRequestSent){
        generateTDMAHeader(PacketType::JOINREQUEST);
        std::vector<uint8_t> joinRequest = TDMAHeader;    //bodyless
        if(send(joinRequest)){  //jumping the send queue 
            joinRequestSent = true;
            timeJoinRequestSent = micros();
            //Serial.println("Join request sent");
        };    
    }


    if (synced && joinRequestSent && !acked){
        getPacket();    //scan for ack
        //Serial.println("Scanning for ack");

        if(_received && static_cast<PacketType>(receivedPacketType) == PacketType::ACK){    //this needs a check for ack being for the join request: use destinetion node but then you have duplication in the rnp packet dunno
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Joined existing TDMA network");
            //Serial.println("Joined existing TDMA network");
            currentMode = TDMA_MODE::TRANSMIT;  // exit discovery, can be any other mode
        }
        else if(_received && static_cast<PacketType>(receivedPacketType) == PacketType::NACK){  
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Joined existing TDMA network");
            //Serial.println("Node already registered");
            txTimewindow = packetInfo;
            currentMode = TDMA_MODE::TRANSMIT;  // exit discovery, can be any other mode
        }

        if (micros() - timeJoinRequestSent > 100000){       //retry
            joinRequestSent = false;
        }

    }
}


void TDMA::sync(){
    // back timestamping when the node was meant to move windows
    timeMovedTimewindow = timePacketReceived - 1.1*calcPacketToF(packetSize);
    
    currTimewindow = packetTimewindow;    // sync local current timewindow to network
    txTimewindow = packetRegNodes;        // send in the n+1th timewindow
    timewindows = packetRegNodes+1;       // update local number of timewindows
    synced = true;                        // syncing complete

    //Serial.println("synced");
    // Serial.println(txTimewindow);
    // Serial.println(currTimewindow);
}



void TDMA::initNetwork(){
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Initializing TDMA Network");
    timewindows = registeredNodes.size() + 1;    //n+1 timewindows
    txTimewindow = 0;  //of this node
    currTimewindow = txTimewindow;
    timeMovedTimewindow = micros();
    //Serial.println(timewindowDuration);
}

void TDMA::generateTDMAHeader(PacketType packettype){
    TDMAHeader = {static_cast<uint8_t>(packettype), static_cast<uint8_t>(registeredNodes.size()), currTimewindow, 255};
}

void TDMA::generateTDMAHeader(PacketType packettype, uint8_t info){
    TDMAHeader = {static_cast<uint8_t>(packettype), static_cast<uint8_t>(registeredNodes.size()), currTimewindow, info};
}


void TDMA::tx(){

    //Serial.println("Entered TX");

    if(_sendBuffer.size() == 0){    //buffer empty

        if (countsNoTx >= maxCountsNoTx){        // node didn't transmit in a long time
            generateTDMAHeader(PacketType::HEARTBEAT);
            std::vector<uint8_t> heartbeatPacket = TDMAHeader;  // bodyless heartbeat packet
            send(heartbeatPacket);
            countsNoTx = 0;      // reset local counter
            txWindowDone = true;       // exit tx mode
            //Serial.println("Send heartbeat");
        }
        else{ 
            //Serial.println("update counter");
            countsNoTx += 1;     // update counter
            txWindowDone = true;       // exit tx mode
        }

    }
    else{                           // buffer not empty
        //Serial.println("Buffer not empty");
        if(countsNoAck > maxCountsNoAck){  // check if exceed limit on resends
            _sendBuffer.pop();      // pop old packet
            countsNoAck = 0;      // reset counter
        }

        if(!packetSent){
            bytes_written = send(_sendBuffer.front());  // send from front of buffer
            if (bytes_written){
                //Serial.println("Packet sent");
                countsNoAck =+ 1;  // just trust me bro, it makes sense
            }
        }
        else{                   // packet has been sent
            getPacket();        // scan for acks
            if (_received && receivedPacketType == PacketType::ACK){ // packet got acked
                _sendBuffer.pop();      // remove packet from send queue
                _currentSendBufferSize -= bytes_written;
                countsNoAck = 0;      // reset counter
                txWindowDone = true;   // exit tx mode
            }
        } 
    }

}

void TDMA::rx(){

    //Serial.println("RX");
    getPacket();    // scan for packets

    if(_received){

        if (static_cast<PacketType>(receivedPacketType) == PacketType::JOINREQUEST){   // handling join request
            //Serial.println("Received join request");

            auto it = find(registeredNodes.begin(), registeredNodes.end(), packetSource);

            if(it == registeredNodes.end()){    // node has not been registered yet
                registeredNodes.push_back(packetSource);   // add to node list

                //ack join request
                generateTDMAHeader(PacketType::ACK);
                std::vector<uint8_t> ackPacket = TDMAHeader;    //bodyless
                send(ackPacket);            
                txWindowDone = true;
            }
            else{   // node is already registered
                // nack join request
                uint8_t requester_txTimewindow = static_cast<uint8_t>(it-registeredNodes.begin());
                generateTDMAHeader(PacketType::NACK, requester_txTimewindow);
                std::vector<uint8_t> ackPacket = TDMAHeader;
                send(ackPacket);            
                txWindowDone = true;               
            }
            
        }
        else{   // handling not a join request
            generateTDMAHeader(PacketType::ACK);
            std::vector<uint8_t> ackPacket = TDMAHeader;    //bodyless
            send(ackPacket);            
            txWindowDone = true;
        }
    }
    //TODO: counter for times havent heard from node
}


void TDMA::calcTimewindowDuration(){
    double maxTframe = 2;                 //assuming 2 seconds
    double clock_drift = 2E-5;            //us/s
    double Tg = maxTframe*clock_drift;    //guard time   
    double ff = 1.1;                      //magic fudge factor
    timewindowDuration = ff*(calcPacketToF(_config.max_payload_length) + calcPacketToF(_config.max_ack_length) + Tg);
}

double TDMA::calcPacketToF(int Lpayload){
    double Rs = _config.bandwidth/pow(2,_config.spreading_factor);
    double CR = _config.cr_denominator - 4;
    double Tsym = 1/Rs;
    double Tpreamble = (_config.preamble_length + 4.25)*Tsym;
    double Tpayload = Tsym*(8 + ceil((8*Lpayload - 4*_config.spreading_factor + 44)/(4*_config.spreading_factor))*(CR + 4));
    return Tpreamble + Tpayload;
}


void TDMA::sendPacket(RnpPacket& data)
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

    std::unique_ptr<std::vector<uint8_t>> serializedPacket = std::make_unique<std::vector<uint8_t>>();
    data.serialize(*serializedPacket);
    generateTDMAHeader(PacketType::NORMAL);
    serializedPacket->insert(serializedPacket->begin(), TDMAHeader.begin(), TDMAHeader.end());
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
    // check if radio is still transmitting
    if (loraRadio.isTransmitting()){
        return;
    }

    int size = loraRadio.parsePacket();

    if (size){
        packetSize = size;
        _received=true;
        //RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Radio Receive");    

        timePacketReceived = micros();
        std::vector<uint8_t> data(packetSize);
        loraRadio.readBytes(data.data(),packetSize);

        //unpacking 4 byte TDMA header
        receivedPacketType = static_cast<PacketType>(data.front());
        //Serial.println(receivedPacketType);
        data.erase(data.begin());
        packetRegNodes = data.front();
        data.erase(data.begin());
        packetTimewindow = data.front();
        data.erase(data.begin());
        if(data.front() != 255){    // non-default value, but this aint great
            packetInfo = data.front();
        }
        data.erase(data.begin());

        // TODO: fix this shit
        // if (currentMode != TDMA_MODE::DISCOVERY && packetRegNodes - static_cast<uint8_t>(registeredNodes.size()) > 0){  //local node list is shorter
        //     registeredNodes.resize(packetRegNodes);
        // }        

        if (!data.empty()){     // packet still has smthing left in it

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
        //RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Radio Send");
        loraRadio.write(data.data(), data.size());
        loraRadio.endPacket(true); // asynchronous send 
        _txDone = false;
        _info.prevTimeSent = millis();
        packetSent = true;
        _received = false;
        acked = false;
        if (static_cast<PacketType>(data.front()) == PacketType::NORMAL){
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Radio RNP packet Send");
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