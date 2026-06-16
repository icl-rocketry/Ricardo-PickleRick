#pragma once


namespace GeneralConfig{
    //Serial baud rate
    static constexpr int SerialBaud = 115200;
    //Serial rx buffer size
    static constexpr int SerialRxSize = 256;

    //I2C frequrency - 4Khz
    static constexpr int I2C_FREQUENCY = 400000;

    // Bench throttle test: entering Flight arms the props and runs the
    // voltage/thrust calibration profile on both motors.
    static constexpr bool ThrottleRampTestEnabled = false;

    // When false, throttle profile values are desired thrust percent.
    // When true, throttle profile values are demanded thrust in Newtons.
    static constexpr bool ThrottleRampProfileCommandsThrustNewtons = false;

    // Fallback location used to compute the magnetometer reference when GPS
    // position is unavailable or the stored reference is invalid.
    //this is referenced to the location of Olis house in London 
    static constexpr double FALLBACK_MAG_REF_LAT_DEG = 51.5750;
    static constexpr double FALLBACK_MAG_REF_LON_DEG = -0.1453;
    static constexpr float  FALLBACK_MAG_REF_ALT_M   = 100.0f; //altitude above sea level 
};
