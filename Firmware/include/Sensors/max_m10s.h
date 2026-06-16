#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <cstdint>

#include "Config/types.h"
#include "Config/systemflags_config.h"
#include "Sensors/sensorStructs.h"


class MAX_M10S
{
public:
    MAX_M10S(TwoWire& wire,
            Types::CoreTypes::SystemStatus_t& systemstatus,
            int ppsPin = -1,
            uint8_t address = M10_I2C_ADDR);

    void setup();
    void update(SensorStructs::GPS_t& data);

private:
    // ── I2C primitives ───────────────────────────────────────────────────────
    // How many bytes the module has queued in its TX buffer for us
    uint16_t bytesAvailable();
    // Pull up to len bytes from the data-stream register (register 0xFF).
    // The module auto-increments its internal read pointer on each byte.
    // Returns the number of bytes actually read.
    uint16_t readStream(uint8_t* buf, uint16_t len);
    // Send a raw buffer to the module (used to deliver UBX command frames)
    void     writeBytes(const uint8_t* buf, uint16_t len);

    // ── PPS capture ─────────────────────────────────────────────────────────
    void setupPpsCapture();
    void copyPpsSnapshot(uint32_t& timestamp_us, uint32_t& count) const;
    void updatePpsStatus(SensorStructs::GPS_t& data, uint32_t now_us) const;
    bool timestampFromPps(uint32_t gnss_time_of_day_ms,
                          uint32_t now_us,
                          uint32_t& timestamp_us);
    static void ARDUINO_ISR_ATTR ppsRiseHandler(void* arg);

    // ── Config helpers ───────────────────────────────────────────────────────
    void cfgValSetU1(uint32_t keyId, uint8_t  value);
    void cfgValSetU2(uint32_t keyId, uint16_t value);
    // Block until an ACK or NAK arrives for the given class/id pair
    bool waitForAck(uint8_t expectedClass, uint8_t expectedId,
                    uint32_t timeoutMs = 500);

    // ── UBX framing ──────────────────────────────────────────────────────────
    void sendUbx(uint8_t msgClass, uint8_t msgId,
                 const uint8_t* payload, uint16_t len);

    // ── NAV-PVT parser ───────────────────────────────────────────────────────
    bool parseByte(uint8_t b);   // returns true when a valid frame is ready
    void unpackPvt();
    
    // ── Members ──────────────────────────────────────────────────────────────
    TwoWire&                          _wire;
    Types::CoreTypes::SystemStatus_t& _systemstatus;
    uint8_t                           _address;
    int                               _ppsPin;
    volatile uint32_t                 _lastPpsTimestampUs = 0;
    volatile uint32_t                 _ppsCount = 0;
    uint32_t                          _mappedPpsGnssSecondMs = 0;
    uint32_t                          _mappedPpsLocalTimestampUs = 0;
    bool                              _mappedPpsValid = false;
    
    // ── NAV-PVT payload length (interface description UBX-21035062) ──────────────
    static constexpr uint16_t NAV_PVT_LEN = 92;
    
    enum class ParseState : uint8_t
    {
        SYNC1, SYNC2, CLASS, ID, LEN_L, LEN_H, PAYLOAD, CK_A, CK_B
    };

    enum class GnssFix : uint8_t
    {
        NO_FIX    = 0,
        DR_ONLY   = 1,
        FIX_2D    = 2,
        FIX_3D    = 3,
        GNSS_DR   = 4,
        TIME_ONLY = 5,
    };

    ParseState _parseState = ParseState::SYNC1;
    uint8_t    _msgClass   = 0;
    uint8_t    _msgId      = 0;
    uint16_t   _payloadLen = 0;
    uint16_t   _payloadIdx = 0;
    uint8_t    _ckA        = 0;
    uint8_t    _ckB        = 0;
    uint8_t    _payload[NAV_PVT_LEN];
    
    struct PvtCache
    {
        uint32_t iTOW;          // GPS time of week / time of day in ms
        int32_t  lat;           // deg * 1e-7
        int32_t  lon;           // deg * 1e-7
        int32_t  hMSL;          // mm above MSL
        int32_t  height;        // mm above ellipsoid
        int32_t  velN;          // mm/s
        int32_t  velE;          // mm/s
        int32_t  velD;          // mm/s
        // derived from v ned and p ned so not getting for computation speed
        // int32_t  gSpeed;     // mm/s ground speed 2D
        // int32_t  headMot;    // deg * 1e-5
        uint32_t hAcc;          // mm
        uint32_t vAcc;          // mm
        uint8_t  numSV;         // Number of Satellites
        GnssFix  fixType;
        bool     gnssFixOk;
        bool     valid;
    } _pvt {};
    
    // ── UBX framing constants ────────────────────────────────────────────────────
    static constexpr uint8_t UBX_SYNC1 = 0xB5;
    static constexpr uint8_t UBX_SYNC2 = 0x62;
    
    // ── Message class / ID ───────────────────────────────────────────────────────
    static constexpr uint8_t UBX_CLASS_NAV  = 0x01;
    static constexpr uint8_t UBX_ID_NAV_PVT = 0x07;
    static constexpr uint8_t UBX_CLASS_CFG  = 0x06;
    static constexpr uint8_t UBX_ID_VALSET  = 0x8A;
    static constexpr uint8_t UBX_CLASS_ACK  = 0x05;
    static constexpr uint8_t UBX_ID_ACK     = 0x01;
    static constexpr uint8_t UBX_ID_NAK     = 0x00;
    
    // ── CFG-VALSET key IDs (I2C port variants) ───────────────────────────────────
    // Note: I2C port keys are offset from UART1 keys — do not mix them up.
    static constexpr uint32_t CFG_MSGOUT_UBX_NAV_PVT_I2C = 0x20910006;
    static constexpr uint32_t CFG_MSGOUT_NMEA_GGA_I2C    = 0x209100BA;
    static constexpr uint32_t CFG_MSGOUT_NMEA_GLL_I2C    = 0x209100C9;
    static constexpr uint32_t CFG_MSGOUT_NMEA_GSA_I2C    = 0x209100BF;
    static constexpr uint32_t CFG_MSGOUT_NMEA_GSV_I2C    = 0x209100C4;
    static constexpr uint32_t CFG_MSGOUT_NMEA_RMC_I2C    = 0x209100AB;
    static constexpr uint32_t CFG_MSGOUT_NMEA_VTG_I2C    = 0x209100B0;
    static constexpr uint32_t CFG_RATE_MEAS               = 0x30210001;
    
    // ── I2C interface registers ──────────────────────────────────────────────────
    // The M10 exposes a DDC (I2C) interface with three logical registers:
    //   0xFD  high byte of bytes-available count
    //   0xFE  low  byte of bytes-available count  (read both in one transaction)
    //   0xFF  data stream — sequential reads return queued output bytes
    static constexpr uint8_t M10_I2C_ADDR      = 0x42;
    static constexpr uint8_t REG_BYTES_AVAIL_H = 0xFD;
    static constexpr uint8_t REG_DATA_STREAM   = 0xFF;

    // ── PPS timing constants ─────────────────────────────────────────────────
    static constexpr uint32_t PPS_PERIOD_US = 1000000UL;
    static constexpr uint32_t PPS_FRESH_TIMEOUT_US = 1500000UL;
    static constexpr uint32_t PPS_FUTURE_TOLERANCE_US = 5000UL;
    
    // Max bytes to pull per readStream() call (Wire buffer is 32 bytes on AVR,
    // 128 on ESP32 — use a safe chunk size for burst reads)
    static constexpr uint8_t I2C_CHUNK = 32;
    static constexpr uint8_t I2C_CHUNKS_PER_UPDATE = 4;
};
