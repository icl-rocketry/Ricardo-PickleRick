/*
**********************
* PINS               *
**********************
 */
#pragma once

namespace PickleRickV1Pins{
    static constexpr int BaroCs = 2;
    static constexpr int LoraReset = 4;
    static constexpr int LoraCs = 5;

    static constexpr int H_MISO = 12;
    static constexpr int H_MOSI = 13;
    static constexpr int H_SCLK = 14;

    static constexpr int MagCs = 15;
    static constexpr int ImuCs_1 = 16;
    static constexpr int ImuCs_2 = 17;

    static constexpr int V_SCLK = 18;
    static constexpr int V_MISO = 19;
    static constexpr int _SDA = 21;
    static constexpr int _SCL = 22;
    static constexpr int V_MOSI = 23;

    static constexpr int SdCs_1 = 25;
    static constexpr int SdCs_2 = 26;
    static constexpr int Buzzer = 27;


    static constexpr int TxCan = 32;
    static constexpr int RxCan = 33;

    static constexpr int SdDet_1 = 34;
    static constexpr int SdDet_2 = 35;
    static constexpr int BattVolt = 36;
    static constexpr int LoraInt = 39;

    //PCA9634 Mapping -> NB this does not match schmatic or silkscreen (will update in v3 pickle)
    static constexpr int Ch0Fire = 1;
    static constexpr int Ch1Fire = 3;
    static constexpr int Ch2Fire = 5;
    static constexpr int Ch3Fire = 7;

    static constexpr int Ch0Cont = 0;
    static constexpr int Ch1Cont = 2;
    static constexpr int Ch2Cont = 4;
    static constexpr int Ch3Cont = 6;
    
};

namespace PickleRickV2Pins{
    static constexpr int BaroCs = 14;

    static constexpr int LoraReset = 9;
    static constexpr int LoraCs = 10;

    static constexpr int H_MISO = 34;
    static constexpr int H_MOSI = 33;
    static constexpr int H_SCLK = 35;

    static constexpr int MagCs = 13;
    static constexpr int ImuCs_1 = 12;
    static constexpr int ImuCs_2 = 11;

    static constexpr int V_SCLK = 37;
    static constexpr int V_MISO = 38;
    static constexpr int V_MOSI = 36;

    static constexpr int _SDA = 21;
    static constexpr int _SCL = 26;
    

    static constexpr int SdCs_1 = 4;
    static constexpr int SdCs_2 = 8;

    static constexpr int Buzzer = 1;

    static constexpr int TxCan = 18;
    static constexpr int RxCan = 17;

    static constexpr int SdDet_1 = 6;
    static constexpr int SdDet_2 = 7;

    static constexpr int BattVolt = 2;
    static constexpr int LoraInt = 5;

    //PCA9634 Mapping -> NB this does not match schmatic or silkscreen (will update in v3 pickle)
    static constexpr int Ch0Fire = 1;
    static constexpr int Ch1Fire = 3;
    static constexpr int Ch2Fire = 5;
    static constexpr int Ch3Fire = 7;

    static constexpr int Ch0Cont = 0;
    static constexpr int Ch1Cont = 2;
    static constexpr int Ch2Cont = 4;
    static constexpr int Ch3Cont = 6;

    //PCA9685 Mapping
    static constexpr int servo0pin = 7;
    static constexpr int servo1pin = 8;
    static constexpr int servo2pin = 9;
    static constexpr int servo3pin = 10;

};

namespace PickleRickV3Pins{
    //Buzzer
    static constexpr int Buzzer = 1;

    //LoRa
    static constexpr int LoraReset = 9;
    static constexpr int LoraCs = 10;
    static constexpr int LoraInt = 4;

    //VSPI
    static constexpr int V_SCLK = 37;
    static constexpr int V_MISO = 38;
    static constexpr int V_MOSI = 36;

    //HSPI
    static constexpr int H_MISO = 34;
    static constexpr int H_MOSI = 33;
    static constexpr int H_SCLK = 35;

    //Interial Sensors
    static constexpr int MagCs = 13;
    static constexpr int ImuCs_1 = 12;
    static constexpr int ImuCs_2 = 11;
    static constexpr int BaroCs = 14;
    static constexpr int PPS = 39;

    //gpio -> doubles as JTAG PINS
    static constexpr int GPIO0 = 40;
    static constexpr int GPIO1 = 41;
    static constexpr int GPIO2 = 42;

    //ABUS
    static constexpr int ABUS_RX = 48;
    static constexpr int ABUS_TX = 47;

    //I2C
    static constexpr int _SDA = 21;
    static constexpr int _SCL = 26;
    
    //SD Storage
    static constexpr int SdCs_1 = 8;
    static constexpr int SdCs_2 = 7;
    static constexpr int SdDet_1 = 5;
    static constexpr int SdDet_2 = 6;
  
    //CAN
    static constexpr int TxCan = 18;
    static constexpr int RxCan = 17;

    //VRail Sense
    // static constexpr int BattVolt = 2;
    static constexpr int LogicVolt = 2;
    static constexpr int DepVolt = 16;
    static constexpr int DepSwitch = 15;
    
    //PCA9634 Mapping 
    static constexpr int Ch0Fire = 1;
    static constexpr int Ch1Fire = 3;
    static constexpr int Ch2Fire = 5;
    static constexpr int Ch3Fire = 7;

    static constexpr int Ch0Cont = 0;
    static constexpr int Ch1Cont = 2;
    static constexpr int Ch2Cont = 4;
    static constexpr int Ch3Cont = 6;

    //PCA9685 Mapping
    static constexpr int servo0pin = 7;
    static constexpr int servo1pin = 8;
    static constexpr int servo2pin = 9;
    static constexpr int servo3pin = 10;

};

#ifdef CONFIG_IDF_TARGET_ESP32S3
namespace PinMap = PickleRickV2Pins;
#else
namespace PinMap = PickleRickV1Pins;
#endif



