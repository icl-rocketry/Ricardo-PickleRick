#pragma once

namespace PickleRickV3Pins{
    //Buzzer
    static constexpr int Buzzer = 1;

    //LoRa
    static constexpr int LoraReset = 9;
    static constexpr int LoraCs = 10;
    static constexpr int LoraInt = 4;
    static constexpr int LoraGPIO = 40;


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