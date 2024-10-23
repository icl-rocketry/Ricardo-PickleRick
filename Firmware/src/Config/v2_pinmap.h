#pragma once

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

    static constexpr int LogicVolt = 2;
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
