#include "sx1280.h"

// SPI
#define LORA_DEFAULT_SPI           SPI
#define LORA_DEFAULT_SPI_FREQUENCY 8E6 
#define LORA_DEFAULT_SS_PIN        10
#define LORA_DEFAULT_RESET_PIN     9
#define LORA_DEFAULT_DIO0_PIN      2

// Register Address Map

#define RxGain 0x891
#define ManualGain 0x895
#define LNAGAIN 0x89E
#define LNAGainControl 0x89F

#define SynchPeakAttenuation 0x8C2
#define PayloadLength 0x901
#define LoraHeaderMode 0x903

#define RangingRequestByte3 0x912
#define RangingRequestByte2 0x913
#define RangingRequestByte1 0x914
#define RangingRequestByte0 0x915

#define RangingDeviceByte3 0x916
#define RangingDeviceByte2 0x917
#define RangingDeviceByte1 0x918
#define RangingDeviceByte0 0x919

#define RangingFilterWindowSize 0x91E
#define ResetRangingFilter 0x923
#define RangingResultMUX 0x924

#define SFAdditionalConfig 0x925

#define RangingCalibrationByte2 0x92B
#define RangingCalibrationByte1 0x92C
#define RangingCalibrationByte0 0x92D

#define RangingIDCheckLength 0x931

#define FrequencyErrorCorection0x93C
#define CadDetPeak 0x942
#define LoraSyncWord 0x944

// IRQ Source Bits

#define TxDone 0
#define RxDone 1
#define SyncWordValid 2
#define SyncWordError 3
#define HeaderValid 4
#define HeaderError 5
#define CrcError 6
#define RangingSlaveResponseDone 7
#define RangingSlaveRequestDiscard 8
#define RangingMasterResultValid 9
#define RangingMasterTimeout 10
#define RangingSlaveRequestValid 11
#define CadDone 12
#define CadDetected 13
#define RxTxTimeout 14
#define PreambleDetected 15
#define AdvancedRangingDone 15


sx1280::sx1280() :
  _spiSettings(LORA_DEFAULT_SPI_FREQUENCY, MSBFIRST, SPI_MODE0),
  _spi(&LORA_DEFAULT_SPI),
  _ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN),
  _frequency(0),
  _packetIndex(0),
  _implicitHeaderMode(0),
  _onReceive(NULL),
  _onCadDone(NULL),
  _onTxDone(NULL)
{}


void sx1280::beginSPI(){

  // setup pins
  pinMode(_ss, OUTPUT);
  // set SS high
  digitalWrite(_ss, HIGH);

  if (_reset != -1) {
    pinMode(_reset, OUTPUT);

    // perform reset
    digitalWrite(_reset, LOW);
    delay(10);
    digitalWrite(_reset, HIGH);
    delay(10);
  }

  // start SPI
  _spi->begin();
}

#define SleepConfig0 0 //RAM flushed
#define SleepConfig1 1 //RAM retained
#define SleepConfig2 2 //Data buffer retained

void sx1280::SetSleep(int mode){

  singleTransfer(0x84, mode);
}

void sx1280::endSPI()
{
  // put in sleep mode
  SetSleep(SleepConfig2);

  // stop SPI
  _spi->end();
}

void sx1280::setPins(int ss, int reset, int dio0)
{
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
}

void sx1280::setSPI(SPIClass& spi)
{
  _spi = &spi;
}


int sx1280::parsePacket()
{
  rxSetup();
  GetRxBufferStatus();
  return static_cast<int>(_rxPayloadLength);
}














void sx1280::GetStatus(){

  singleTransfer(0xC0);

}

void sx1280::WriteRegister(uint16_t address, uint32_t value) {

    singleTransfer(0x18, address, value);

}

void sx1280::ReadRegister(uint16_t address) {
  
    spiReturn = singleTransfer(0x19, address, 0x00,0x00);
    _register = spiReturn[4];

}

void sx1280::WriteBuffer(uint8_t &offset, std::vector<uint8_t> &payload){

    singleTransfer(0x1A, offset, payload);
    
    // offset loops back to 0 after 255 (FIX)
    if (_offset < 255){
        _offset++;
    }
    else{
        _offset = 0;
    }

}

void sx1280::ReadBuffer(uint8_t &offset){

    spiReturn = singleTransfer(0x1B, offset, 0x00,0x00);
    _buffer = spiReturn[3];
}


#define STDBY_RC 0 //Device running on RC 13MHz, set STDBY_RC mode
#define STDBY_X0SC 1 //Device running on XTAL 52MHz, set STDBY_XOSC mode

void sx1280::SetStandby(int mode){

  // set device in either STDBY_RC or STDBY_X0SC mode

  singleTransfer(0x80, mode);
}

void sx1280::SetFs(){

  singleTransfer(0xC1);
}


// Timeout Step  Timeout duration = periodBase*periodBaseCount
#define periodBase0 0x00 //15.625us
#define periodBase1 0x01 //62.5us
#define periodBase2 0x02 //1ms
#define periodBase3 0x03 //4ms

// Timeout Mode
#define notimeout 0x0000 
#define timeout 0x0001
#define rxContinous 0xFF //Device remains in Rx mode when sending two bytes

void sx1280::SetTx(int periodBase){

    ClearIrqStatus(0xFF);

    singleTransfer(0x83, periodBase, 0x00, 0x00);
}

void sx1280::SetRx(int periodBase, int timeoutMode){

  ClearIrqStatus(0x01);
  ClearIrqStatus(0xE);

  switch(timeoutMode){

    case(notimeout):{
      singleTransfer(0x82,periodBase,0x00,0x00);
    }

    case(timeout):{
      singleTransfer(0x82,periodBase,0x00,0x01);
    }

    case(rxContinous):{
      singleTransfer(0x82,periodBase,0xFF,0xFF);
    }

  }
}



void sx1280::SetRxDutyCycle(int periodBase){

  singleTransfer(0x94, periodBase, 0x00, 0xAF,0x00,0xFA);
}
 



void sx1280::SetLongPreamble(){

  singleTransfer(0x9B,0x01);

}


void sx1280::SetCad(){

  singleTransfer(0xC5);

}

void sx1280::SetTxContinuousWave(){

  singleTransfer(0xD1);

}

void sx1280::SetTxContinousPreamble(){

  singleTransfer(0xD2);

}

void sx1280::SetAutoTx(){

  singleTransfer(0x98, 0x00, 0x5C);
}




#define autoFsDisable 0x00
#define autoFsEnable 0x01

void sx1280::SetAutoFs(int enable){

  singleTransfer(0x9E, enable); //Enable- 0x01, Disable- 0x00
}




#define Packet_Type_Lora 0x01
#define Packet_Type_Ranging 0x02

void sx1280::SetPacketType(int packetType){

  singleTransfer(0x8A, packetType); // 0x01 for LoRa, 0x02 for Ranging
}




void sx1280:: GetPacketType(){

  spiReturn = singleTransfer(0x03, 0x00, 0x00);
  _packetType = spiReturn[2];
}


void sx1280::SetRfFrequency(uint32_t frequency){

  singleTransfer(0x86, frequency);

    _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  singleTransfer((uint8_t)(frf >> 16), (uint8_t)(frf >> 8),(uint8_t)(frf >> 0));

}

#define RADIO_RAMP_02_US 0x00 
#define RADIO_RAMP_04_US 0x20 
#define RADIO_RAMP_06_US 0x40 
#define RADIO_RAMP_08_US 0x60
#define RADIO_RAMP_10_US 0x80 
#define RADIO_RAMP_12_US 0xA0 
#define RADIO_RAMP_16_US 0xC0
#define RADIO_RAMP_20_US 0xE0 

void sx1280::SetTxParams(int power, int rampTime){

  singleTransfer(0x8E, power +18, rampTime);  //0x1F (31) for 13dBm, (between 0 - 31)
}   

// Channel Activity Detection symbols used (higher false detection risk for 1 and 2 symbols)
#define LORA_CAD_01_SYMBOL 0x00 //1
#define LORA_CAD_02_SYMBOLS 0x20 //2
#define LORA_CAD_04_SYMBOLS 0x40 //4
#define LORA_CAD_08_SYMBOLS 0x60 //8
#define LORA_CAD_16_SYMBOLS 0x80 //16

void sx1280::SetCadParams(uint8_t cadSymbolNum){

  singleTransfer(0x88, cadSymbolNum); 

}

void sx1280::SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress){

  singleTransfer(0x8F, txBaseAddress, rxBaseAddress);
}


// Modulation Parameters

//Spreading Factor
#define LORA_SF_5 0x50 //5
#define LORA_SF_6 0x60 //6
#define LORA_SF_7 0x70 //7
#define LORA_SF_8 0x80 //8
#define LORA_SF_9 0x90 //9
#define LORA_SF_10 0xA0 //10
#define LORA_SF_11 0xB0 //11
#define LORA_SF_12 0xC0 //12

//Bandwidth
#define LORA_BW_1600 0x0A //1625.0
#define LORA_BW_800 0x18 //812.5
#define LORA_BW_400 0x26 //406.25
#define LORA_BW_200 0x34 //203.125

// Coding Rate (*Interleaving increases robustness to burst interference and doppler events)
#define LORA_CR_4_5 0x01 //4/5
#define LORA_CR_4_6 0x02 //4/6
#define LORA_CR_4_7 0x03 //4/7
#define LORA_CR_4_8 0x04 //4/8
#define LORA_CR_LI_4_5 0x05 //4/5*
#define LORA_CR_LI_4_6 0x06 //4/6*
#define LORA_CR_LI_4_8 0x07 //4/8*

void sx1280::SetModulationParams(uint8_t modParam1, uint8_t modParam2, uint8_t modParam3){

  singleTransfer(0x8B,modParam1,modParam2,modParam3);
}

// packetParam1: Preamble Length


//packetParam2: Header Type
#define EXPLICIT_HEADER 0x00 //EXPLICIT HEADER
#define IMPLICIT_HEADER 0x80 //IMPLICIT HEADER

//packetParam3: Payload Length
#define PAYLOAD_LENGTH 0x12 //Payload Length 

//packetParam4: CRC Enable
#define CRC_ENABLE 0x20 //CRC ENABLE
#define CRC_DISABLE 0x00 //CRC DISABLE

//packetParam5: IQ Inversion
#define LORA_IQ_INVERTED 0x00 //Swapped IQ
#define LORA_IQ_STD 0x40 //Standard IQ

void sx1280::SetPacketParams(uint8_t packetParam1, uint8_t packetParam2, uint8_t packetParam3, uint8_t packetParam4, uint8_t packetParam5){

  singleTransfer(0x8C,packetParam1,packetParam2, packetParam3, packetParam4, packetParam5, 0x00, 0x00);
  
  if(packetParam2 == EXPLICIT_HEADER){_implicitHeaderMode = 1;}
  else{_implicitHeaderMode = 0;};
}

void sx1280::GetRxBufferStatus(){

  spiReturn = singleTransfer(0x17, 0x00,0x00,0x00);
  _rxPayloadLength = spiReturn[2];
  _rxStartBufferPointer = spiReturn[3];

}

void sx1280::GetPacketStatus(){

  spiReturn = singleTransfer(0x1D,0x00,0x00,0x00,0x00,0x00,0x00); //(opCode, status, rssi sync, snrPkt, -, -, -)
  _rssiSync = spiReturn[2];
  snrPkt = spiReturn[3];

} 


int8_t sx1280::GetRssiInst(){

    spiReturn = singleTransfer(0x1F,0x00,0x00);
    _rssiInst = spiReturn[2];
    return _rssiInst;
}

void sx1280::SetDioIrqParams(){

  singleTransfer(0x8D,0x40,0x23,0x00,0x01,0x00,0x02,0x40,0x20); // Change later for required params

}

void sx1280::GetIrqStatus(){

  spiReturn = singleTransfer(0x15, 0x00, 0x00, 0x00);
  _irqstatus = static_cast<uint16_t>(spiReturn[2]) << 8 | static_cast<uint16_t>(spiReturn[3]);

}

void sx1280::ClearIrqStatus(uint16_t _irqFlag){

  uint8_t irqbyte1 = static_cast<uint8_t>(_irqFlag >> 8);
  uint8_t irqbyte2 = static_cast<uint8_t>(_irqFlag & 0xFF);
  singleTransfer(0x97,irqbyte1, irqbyte2);

}

void sx1280::SetRegulatorMode(){

  singleTransfer(0x96,0x01);  //(opcode,regMode)
  }

void sx1280::SetSaveContext(){

  singleTransfer(0xD5);

}

void sx1280::SetRangingRole(){

  singleTransfer(0xA3, 0x00);  //(opcode, 0x00(slave)  or 0x01(master))

}

void sx1280::SetAdvancedRanging(){

  singleTransfer(0x9A, 0x00);  //(opCode, 0x00(disable) or 0x01(enable))

}

// LORA TRANSCEIVER OPERATION

void sx1280::setup(){

  SetStandby(STDBY_RC);
  SetPacketType(Packet_Type_Lora);  //set to lora
  SetRfFrequency(2400000000); //set to 2.4GHz
  SetBufferBaseAddress(_txBaseAddress,_rxBaseAddress);
  SetModulationParams(LORA_SF_7,LORA_BW_800,LORA_CR_4_5);
  WriteRegister(0x925,0x37);
  SetPacketParams(0x0C, IMPLICIT_HEADER ,PAYLOAD_LENGTH,CRC_ENABLE,LORA_IQ_STD);  

}

void sx1280::txSetup(std::vector<uint8_t> &data){

  SetTxParams(13,RADIO_RAMP_02_US);
  WriteBuffer(_offset,data);
  SetDioIrqParams();
  SetTx(periodBase2);
  ClearIrqStatus(0xFFFF);

}

void sx1280::rxSetup(){

  SetDioIrqParams();
  SetRx(periodBase2,rxContinous);
  GetPacketStatus();
  ClearIrqStatus(0XFFFF);
  GetRxBufferStatus();
  ReadBuffer(_offset);

}





  

