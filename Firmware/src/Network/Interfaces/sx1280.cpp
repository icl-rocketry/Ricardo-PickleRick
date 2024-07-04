#include "sx1280.h"

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


// Modes

#define SleepConfig0 0 //RAM flushed
#define SleepConfig1 1 //RAM retained
#define SleepConfig2 2 //Data buffer retained

#define STDBY_RC 0 //Device running on RC 13MHz, set STDBY_RC mode
#define STDBY_X0SC 1 //Device running on XTAL 52MHz, set STDBY_XOSC mode



sx1280::sx1280() {
  // Constructor


}


void sx1280::GetStatus(){

  singleTransfer(0xC0);

}

void sx1280::WriteRegister(uint16_t address, uint32_t value) {

    singleTransfer(0x18, address, value);

}

void sx1280::ReadRegister(uint16_t address) {
  
    singleTransfer(0x19, address, 0x00);  
}

void sx1280::WriteBuffer(uint8_t &offset, uint32_t value){

    singleTransfer(0x1A, offset, value);

    // offset loops back to 0 after 255
    if (_offset < 255){
        _offset++;
    }
    else{
        _offset = 0;
    }

}

void sx1280::ReadBuffer(uint8_t &offset){

    singleTransfer(0x1B, offset, 0x00);

}


void sx1280::SetSleep(int mode){

  singleTransfer(0x84, mode);
}

void sx1280::SetStandby(int mode){

  // set device in either STDBY_RC or STDBY_X0SC mode

  singleTransfer(0x80, mode);
}

void sx1280::SetFs(){

  singleTransfer(0xC1);
}

void sx1280::SetTx(){

    ClearIqrStatus();

    singleTransfer(0x83, 0x00, 0x00, 0x00);
}

void sx1280::SetRx(){

  ClearIqrStatus();

  singleTransfer(0x82, 0x03, 0x00, 0x00);
}

void sx1280::SetRxDutyCycle(){

  singleTransfer(0x94, 0x03, 0x00, 0x00);
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

void sx1280::SetAutoFs(){

  singleTransfer(0x9E, 0x00); //Enable- 0x01, Disable- 0x00
}

void sx1280::SetPacketType(){

  singleTransfer(0x8A, 0x01); // 0x01 for LoRa, 0x02 for Ranging
}


void sx1280:: GetPacketType(){

  singleTransfer(0x03, 0x00, 0x00);
}

void sx1280::SetRfFrequency(){

  singleTransfer(0x86, 0xB8, 0x9D, 0x89);
}

void sx1280::SetTxParams(){

  singleTransfer(0x8E, 0x1F, 0xE0);  //0x1F (31) for 13dBm, (between 0 - 31)

}

void sx1280::SetCadParams(){

  singleTransfer(0x88, 0x80); 

}

void sx1280::SetBufferBaseAddress(){

  singleTransfer(0x8F, 0x80, 0x00); // (opcode, txBaseAdress, rxBaseAdress)
}

void sx1280::SetModulationParams(){

  singleTransfer(0x8B,0x70,0x0A,0x01);//(opCode,SF,BW,coding rate)
}

void sx1280::SetPacketParams(){

  singleTransfer(0x8C,0x0C,0x00,0x80,0x20,0x40,0x00,0x00); //(opCode,PreambleLength ,HeaderType, Payload Length, CRC, InvertIq, notUsed, notUsed)
}

void sx1280::GetRxBufferStatus(){

  singleTransfer(0x17, 0x00,0x00,0x00);

}

void sx1280::GetPacketStatus(){

  singleTransfer(0x1D,0x00,0x00,0x00,0x00,0x00,0x00); //(opCode, status, rssi sync, snrPkt, -, -, -)

}

void sx1280::GetRssiInst(){

    singleTransfer(0x1F,0x00,0x00);
}

void sx1280::SetDioIqrParams(){

  singleTransfer(0x8D,0x40,0x23,0x00,0x01,0x00,0x02,0x40,0x20); // Change later for required params

}

void sx1280::GetIqrStatus(){

  singleTransfer(0x15, 0x00, 0x00, 0x00);

}

void sx1280::ClearIqrStatus(){

  singleTransfer(0x97,0XFF, 0xFF);

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

void sx1280::setup(){

  SetStandby(STDBY_RC);
  SetPacketType(); //preset to lora
  SetRfFrequency(); //preset to 2.4GHz



}



// LORA TRANSCEIVER OPERATION

void sx1280::settings(){

  SetStandby(STDBY_RC);
  SetPacketType();  //set to lora
  SetRfFrequency(); //set to 2.4GHz
  SetBufferBaseAddress();
  SetModulationParams();
  WriteRegister(0x925,0x37);
  SetPacketParams();  

}

void sx1280::txSetup(){

  SetTxParams();
  WriteBuffer(_offset,_data);
  

}



template <typename T, typename... Args> void sx1280::singleTransfer(T first, Args... args){

  digitalWrite(_ss, LOW);

  _spi->beginTransaction(_spiSettings);

   for(uint8_t i = 0; i<size(first); i++){
      _spi->transfer((first >> (i*8)) & 0xFF);
    }

   for(uint8_t j = 0; j<size(args...); j++){
      _spi->transfer((args... >> (j*8)) & 0xFF);
    }

  _spi->endTransaction();
  digitalWrite(_ss, HIGH);

}


  

