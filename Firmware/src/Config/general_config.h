#pragma once


namespace GeneralConfig {
    //Serial baud rate
    static constexpr int SerialBaud = 115200;
    //Serial rx buffer size
    static constexpr int SerialRxSize = 256;

    //I2C frequrency - 4Khz
    static constexpr int I2C_FREQUENCY = 400000;

    /*---------- ---------- TOASTER CONFIGS ---------- ----------*/

    // ----- State configs -----

    /// @brief Delay in milliseconds after the deploy state is
    /// reached before extending the grid fin assembly.
    static constexpr unsigned int DeployDelayMs = 3000;

    /// @brief The maximal amount of time that the zeroing state will try
    /// to zero the actuator. After this time, if the endstop is not reached
    /// then default state will be entered and an error flag set.
    static constexpr uint64_t MaxZeroTimeMs = 10000;

    // ----- Comms configs -----

    /// @brief Min delay between commands being set to the actuators.
    static constexpr uint64_t CommandDeltaMs = 500;

    enum class NetworkConfig : uint8_t {
        CHAD_MASTER = 150,
        CHAD_SLAVE = 151
    };

    // ----- Control configs -----

    /// @brief PID roll control constant gain.
    static constexpr double PIDRollKP = 5;

    /// @brief PID roll control integral gain.
    static constexpr double PIDRollKI = 1;

    /// @brief PID roll control derivative gain.
    static constexpr double PIDRollKD = 1;

    /// @brief PID roll control integral limit.
    static constexpr double PIDRollILimit = 10;

    /// @brief PID roll control derivative limit.
    static constexpr double PIDRollDLimit = 10;

    // ----- Actuator configs -----

    static constexpr uint16_t act0ZeroAngle = 900;
    static constexpr uint16_t act1ZeroAngle = 900;
    static constexpr uint16_t act2ZeroAngle = 900;
};







