#include "Sensors/max_m10s.h"

// ── Constructor ───────────────────────────────────────────────────────────────

MAX_M10S::MAX_M10S(TwoWire& wire,
                 Types::CoreTypes::SystemStatus_t& systemstatus,
                 uint8_t address)
    : _wire(wire),
      _systemstatus(systemstatus),
      _address(address)
{
}

// ── Public ────────────────────────────────────────────────────────────────────

void MAX_M10S::setup()
{
    // Verify the module is alive by checking we can read the byte-count
    // registers without a bus error.
    _wire.beginTransmission(_address);
    _wire.write(REG_BYTES_AVAIL_H);
    if (_wire.endTransmission(false) != 0)
    {
        _systemstatus.newFlag(SYSTEM_FLAG::ERROR_GPS,
                              "MAX-M10S not found on I2C bus");
        
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(
        "MAX_M10S not on i2c bus");
        
        return;
    }

    // Drain any startup NMEA the module may already have queued
    delay(500);  // give module more time to boot and queue NMEA
    uint8_t discard[I2C_CHUNK];
    uint32_t drainDeadline = millis() + 2000;
    while (millis() < drainDeadline)
    {
        while (bytesAvailable()) { readStream(discard, I2C_CHUNK); }
        delay(10);
    }

    // ── Disable all default NMEA output on the I2C port ──────────────────
    cfgValSetU1(CFG_MSGOUT_NMEA_GGA_I2C, 0);
    if (!waitForAck(UBX_CLASS_CFG, UBX_ID_VALSET))
    {
        _systemstatus.newFlag(SYSTEM_FLAG::ERROR_GPS,
                              "MAX-M10S: no ACK disabling GGA");

        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(
        "MAX_M10S disabling GGA");
        
        return;
    }

    cfgValSetU1(CFG_MSGOUT_NMEA_GLL_I2C, 0);
    waitForAck(UBX_CLASS_CFG, UBX_ID_VALSET);

    cfgValSetU1(CFG_MSGOUT_NMEA_GSA_I2C, 0);
    waitForAck(UBX_CLASS_CFG, UBX_ID_VALSET);

    cfgValSetU1(CFG_MSGOUT_NMEA_GSV_I2C, 0);
    waitForAck(UBX_CLASS_CFG, UBX_ID_VALSET);

    cfgValSetU1(CFG_MSGOUT_NMEA_RMC_I2C, 0);
    waitForAck(UBX_CLASS_CFG, UBX_ID_VALSET);

    cfgValSetU1(CFG_MSGOUT_NMEA_VTG_I2C, 0);
    waitForAck(UBX_CLASS_CFG, UBX_ID_VALSET);

    // ── Set measurement rate ───────────────────────────────────────────────
    // 100 ms = 10 Hz.  Adjust as needed (200 = 5 Hz, 1000 = 1 Hz).
    cfgValSetU2(CFG_RATE_MEAS, 100);
    if (!waitForAck(UBX_CLASS_CFG, UBX_ID_VALSET))
    {
        _systemstatus.newFlag(SYSTEM_FLAG::ERROR_GPS,
                              "MAX-M10S: no ACK setting nav rate");

        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(
        "MAX_M10S no ACK setting nav rate");

        return;
    }

    // ── Enable UBX-NAV-PVT at 1 frame per epoch on the I2C port ──────────
    cfgValSetU1(CFG_MSGOUT_UBX_NAV_PVT_I2C, 1);
    if (!waitForAck(UBX_CLASS_CFG, UBX_ID_VALSET))
    {
        _systemstatus.newFlag(SYSTEM_FLAG::ERROR_GPS,
                              "MAX-M10S: no ACK enabling NAV-PVT");

        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(
        "MAX_M10S no ACK enabling NAV-PVT");

        return;
    }

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("GPS Initialized");
}

void MAX_M10S::update(SensorStructs::GPS_t& data)
{
    // Ask the module how many bytes it has queued, then pull them all and
    // feed each one through the parser.  parseByte() triggers unpackPvt()
    // internally the moment a valid NAV-PVT frame completes.
    uint16_t avail = bytesAvailable();
    if (avail == 0) { return; }

    uint8_t buf[I2C_CHUNK];
    while (avail > 0)
    {
        uint16_t chunk = (avail > I2C_CHUNK) ? I2C_CHUNK : avail;
        uint16_t got   = readStream(buf, chunk);
        for (uint16_t i = 0; i < got; ++i) { parseByte(buf[i]); }
        avail -= got;
    }

    data.sat      = _pvt.numSV;
    
    if (!_pvt.valid) { return; }

    data.latitude      = _pvt.lat;
    data.longitude      = _pvt.lon;
    data.altitude = static_cast<float>(_pvt.hMSL)    * 1e-3f;

    data.v_n      = static_cast<float>(_pvt.velN)    * 1e-3f;
    data.v_e      = static_cast<float>(_pvt.velE)    * 1e-3f;
    data.v_d      = static_cast<float>(_pvt.velD)    * 1e-3f;

    data.hAcc     = static_cast<float>(_pvt.hAcc)    * 1e-3f;
    data.vAcc     = static_cast<float>(_pvt.vAcc)    * 1e-3f;
    data.fix      = static_cast<uint8_t>(_pvt.fixType);
    data.valid    = _pvt.gnssFixOk;
}

// ── Private ───────────────────────────────────────────────────────────────────

// ── I2C primitives ────────────────────────────────────────────────────────────

uint16_t MAX_M10S::bytesAvailable()
{
    // Registers 0xFD (high) and 0xFE (low) hold a big-endian count of bytes
    // the module has buffered for us.  0xFFFF means no data.
    _wire.beginTransmission(_address);
    _wire.write(REG_BYTES_AVAIL_H);
    if (_wire.endTransmission(false) != 0) { return 0; }

    if (_wire.requestFrom(_address, static_cast<uint8_t>(2)) != 2) { return 0; }

    uint16_t count = (static_cast<uint16_t>(_wire.read()) << 8)
                   |  static_cast<uint16_t>(_wire.read());

    return (count == 0xFFFF) ? 0 : count;
}

uint16_t MAX_M10S::readStream(uint8_t* buf, uint16_t len)
{
    // Point at the data-stream register, then request bytes.
    // The module advances its internal read pointer for every byte clocked out.
    _wire.beginTransmission(_address);
    _wire.write(REG_DATA_STREAM);
    if (_wire.endTransmission(false) != 0) { return 0; }

    uint8_t  req = (len > I2C_CHUNK) ? I2C_CHUNK : static_cast<uint8_t>(len);
    uint8_t  got = _wire.requestFrom(_address, req);
    for (uint8_t i = 0; i < got; ++i) { buf[i] = _wire.read(); }
    return got;
}

void MAX_M10S::writeBytes(const uint8_t* buf, uint16_t len)
{
    // The Wire library has a 32-byte TX buffer on most platforms, so split
    // large writes into chunks.  UBX command frames are small in practice
    // (< 20 bytes for CFG-VALSET with a single key), so this is a precaution.
    uint16_t sent = 0;
    while (sent < len)
    {
        uint16_t chunk = len - sent;
        if (chunk > I2C_CHUNK) { chunk = I2C_CHUNK; }

        _wire.beginTransmission(_address);
        _wire.write(buf + sent, static_cast<uint8_t>(chunk));
        _wire.endTransmission();
        sent += chunk;
    }
}

// ── Config helpers ────────────────────────────────────────────────────────────

void MAX_M10S::cfgValSetU1(uint32_t keyId, uint8_t value)
{
    // CFG-VALSET payload: version(1) layers(1) reserved(2) keyId(4) value(1)
    uint8_t payload[9];
    payload[0] = 0x00;                           // version
    payload[1] = 0x01;                           // layer: RAM
    payload[2] = 0x00;                           // reserved
    payload[3] = 0x00;                           // reserved
    payload[4] = (keyId >>  0) & 0xFF;
    payload[5] = (keyId >>  8) & 0xFF;
    payload[6] = (keyId >> 16) & 0xFF;
    payload[7] = (keyId >> 24) & 0xFF;
    payload[8] = value;
    sendUbx(UBX_CLASS_CFG, UBX_ID_VALSET, payload, sizeof(payload));
}

void MAX_M10S::cfgValSetU2(uint32_t keyId, uint16_t value)
{
    // CFG-VALSET payload: version(1) layers(1) reserved(2) keyId(4) value(2)
    uint8_t payload[10];
    payload[0] = 0x00;
    payload[1] = 0x01;
    payload[2] = 0x00;
    payload[3] = 0x00;
    payload[4] = (keyId >>  0) & 0xFF;
    payload[5] = (keyId >>  8) & 0xFF;
    payload[6] = (keyId >> 16) & 0xFF;
    payload[7] = (keyId >> 24) & 0xFF;
    payload[8] = (value >> 0) & 0xFF;
    payload[9] = (value >> 8) & 0xFF;
    sendUbx(UBX_CLASS_CFG, UBX_ID_VALSET, payload, sizeof(payload));
}

bool MAX_M10S::waitForAck(uint8_t expectedClass, uint8_t expectedId,
                          uint32_t timeoutMs)
{
    // Run a lightweight local parser that only looks for ACK/NAK frames.
    // We poll bytesAvailable() rather than blocking, so the timeout is real.
    enum class AckState : uint8_t
    {
        SYNC1, SYNC2, CLASS, ID, LEN_L, LEN_H, PAYLOAD, CK_A, CK_B
    };

    AckState state       = AckState::SYNC1;
    uint8_t  msgClass    = 0;
    uint8_t  msgId       = 0;
    uint16_t payLen      = 0;
    uint16_t payIdx      = 0;
    uint8_t  ckA         = 0;
    uint8_t  ckB         = 0;
    uint8_t  ackPay[2]   = {};

    uint32_t deadline = millis() + timeoutMs;

    while (millis() < deadline)
    {
        uint16_t avail = bytesAvailable();
        if (avail == 0) { delay(1); continue; }

        uint8_t buf[I2C_CHUNK];
        uint16_t got = readStream(buf, (avail > I2C_CHUNK) ? I2C_CHUNK : avail);

        for (uint16_t i = 0; i < got; ++i)
        {
            uint8_t b = buf[i];

            switch (state)
            {
                case AckState::SYNC1:
                    if (b == UBX_SYNC1) { state = AckState::SYNC2; }
                    break;

                case AckState::SYNC2:
                    state = (b == UBX_SYNC2) ? AckState::CLASS : AckState::SYNC1;
                    break;

                case AckState::CLASS:
                    msgClass = b; ckA = b; ckB = ckA;
                    state = AckState::ID;
                    break;

                case AckState::ID:
                    msgId = b; ckA += b; ckB += ckA;
                    state = AckState::LEN_L;
                    break;

                case AckState::LEN_L:
                    payLen = b; ckA += b; ckB += ckA;
                    state = AckState::LEN_H;
                    break;

                case AckState::LEN_H:
                    payLen |= (static_cast<uint16_t>(b) << 8);
                    ckA += b; ckB += ckA;
                    payIdx = 0;
                    state = AckState::PAYLOAD;
                    break;

                case AckState::PAYLOAD:
                    if (payIdx < 2) { ackPay[payIdx] = b; }
                    ckA += b; ckB += ckA;
                    if (++payIdx >= payLen) { state = AckState::CK_A; }
                    break;

                case AckState::CK_A:
                    if (b != ckA) { state = AckState::SYNC1; break; }
                    state = AckState::CK_B;
                    break;

                case AckState::CK_B:
                    state = AckState::SYNC1;
                    if (b != ckB) { break; }
                    if (msgClass   == UBX_CLASS_ACK
                     && ackPay[0] == expectedClass
                     && ackPay[1] == expectedId)
                    {
                        return (msgId == UBX_ID_ACK);
                    }
                    break;
            }
        }
    }

    return false;
}

// ── UBX framing ───────────────────────────────────────────────────────────────

void MAX_M10S::sendUbx(uint8_t msgClass, uint8_t msgId,
                       const uint8_t* payload, uint16_t len)
{
    // Fletcher-8 checksum over class, id, length (LE), payload
    uint8_t ckA = 0, ckB = 0;
    auto accumulate = [&](uint8_t b) { ckA += b; ckB += ckA; };

    accumulate(msgClass);
    accumulate(msgId);
    accumulate(static_cast<uint8_t>(len & 0xFF));
    accumulate(static_cast<uint8_t>((len >> 8) & 0xFF));
    for (uint16_t i = 0; i < len; ++i) { accumulate(payload[i]); }

    // Build the complete frame in a local buffer and send it in one
    // transaction where possible (avoids the module seeing a partial frame).
    // Max CFG-VALSET with 1 key is 9+6 = 15 bytes — comfortably under I2C_CHUNK.
    uint16_t frameLen = 6 + len + 2;
    uint8_t  frame[64];   // 64 bytes is sufficient for all config messages we send
    if (frameLen > sizeof(frame)) { return; }  // guard against misuse

    frame[0] = UBX_SYNC1;
    frame[1] = UBX_SYNC2;
    frame[2] = msgClass;
    frame[3] = msgId;
    frame[4] = static_cast<uint8_t>(len & 0xFF);
    frame[5] = static_cast<uint8_t>((len >> 8) & 0xFF);
    for (uint16_t i = 0; i < len; ++i) { frame[6 + i] = payload[i]; }
    frame[6 + len]     = ckA;
    frame[6 + len + 1] = ckB;

    writeBytes(frame, frameLen);
}

// ── NAV-PVT parser ────────────────────────────────────────────────────────────

bool MAX_M10S::parseByte(uint8_t b)
{
    switch (_parseState)
    {
        case ParseState::SYNC1:
            if (b == UBX_SYNC1) { _parseState = ParseState::SYNC2; }
            return false;

        case ParseState::SYNC2:
            _parseState = (b == UBX_SYNC2) ? ParseState::CLASS : ParseState::SYNC1;
            return false;

        case ParseState::CLASS:
            _msgClass = b; _ckA = b; _ckB = _ckA;
            _parseState = ParseState::ID;
            return false;

        case ParseState::ID:
            _msgId = b; _ckA += b; _ckB += _ckA;
            _parseState = ParseState::LEN_L;
            return false;

        case ParseState::LEN_L:
            _payloadLen = b; _ckA += b; _ckB += _ckA;
            _parseState = ParseState::LEN_H;
            return false;

        case ParseState::LEN_H:
            _payloadLen |= (static_cast<uint16_t>(b) << 8);
            _ckA += b; _ckB += _ckA;
            _payloadIdx = 0;
            // Only buffer NAV-PVT frames with the expected length;
            // reset for everything else so we don't stall on unknown messages.
            if (_msgClass == UBX_CLASS_NAV
             && _msgId    == UBX_ID_NAV_PVT
             && _payloadLen == NAV_PVT_LEN)
            {
                _parseState = ParseState::PAYLOAD;
            }
            else
            {
                _parseState = ParseState::SYNC1;
            }
            return false;

        case ParseState::PAYLOAD:
            if (_payloadIdx < NAV_PVT_LEN) { _payload[_payloadIdx] = b; }
            _ckA += b; _ckB += _ckA;
            if (++_payloadIdx >= _payloadLen) { _parseState = ParseState::CK_A; }
            return false;

        case ParseState::CK_A:
            if (b != _ckA)
            {
                RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(
                    "MAX_M10S NAV-PVT checksum A mismatch");
                _parseState = ParseState::SYNC1;
                return false;
            }
            _parseState = ParseState::CK_B;
            return false;

        case ParseState::CK_B:
            _parseState = ParseState::SYNC1;
            if (b != _ckB)
            {
                RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(
                    "MAX_M10S NAV-PVT checksum B mismatch");
                return false;
            }
            unpackPvt();
            return true;
    }

    return false;
}

// NAV-PVT byte offsets (interface description UBX-21035062, section NAV-PVT):
//  20     fixType
//  21     flags       bit0 = gnssFixOk
//  23     numSV
//  24-27  lon         deg * 1e-7  (little-endian int32)
//  28-31  lat         deg * 1e-7
//  32-35  height      mm above ellipsoid
//  36-39  hMSL        mm above MSL
//  40-43  hAcc        mm (uint32)
//  44-47  vAcc        mm (uint32)
//  48-51  velN        mm/s
//  52-55  velE        mm/s
//  56-59  velD        mm/s
//  60-63  gSpeed      mm/s  (2D ground speed)
//  64-67  headMot     deg * 1e-5

void MAX_M10S::unpackPvt()
{
    auto i32 = [&](uint8_t o) -> int32_t
    {
        return static_cast<int32_t>(
            (uint32_t)_payload[o+3] << 24 |
            (uint32_t)_payload[o+2] << 16 |
            (uint32_t)_payload[o+1] <<  8 |
            (uint32_t)_payload[o+0]);
    };
    auto u32 = [&](uint8_t o) -> uint32_t
    {
        return (uint32_t)_payload[o+3] << 24 |
               (uint32_t)_payload[o+2] << 16 |
               (uint32_t)_payload[o+1] <<  8 |
               (uint32_t)_payload[o+0];
    };

    _pvt.fixType   = static_cast<GnssFix>(_payload[20]);
    _pvt.gnssFixOk = (_payload[21] & 0x01) != 0;
    _pvt.numSV     = _payload[23];
    _pvt.lon       = i32(24);
    _pvt.lat       = i32(28);
    _pvt.height    = i32(32);
    _pvt.hMSL      = i32(36);
    _pvt.hAcc      = u32(40);
    _pvt.vAcc      = u32(44);
    _pvt.velN      = i32(48);
    _pvt.velE      = i32(52);
    _pvt.velD      = i32(56);
    // derived from v ned and p ned so not getting for computation speed
    // _pvt.gSpeed    = i32(60); 
    // _pvt.headMot   = i32(64); 
    _pvt.valid     = true;

    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(
    // "MAX_M10S fix=" + std::to_string(static_cast<uint8_t>(_pvt.fixType)) +
    // " ok="  + std::to_string(_pvt.gnssFixOk) +
    // " sv="  + std::to_string(_pvt.numSV) +
    // " lat=" + std::to_string(static_cast<float>(_pvt.lat)  * 1e-7f) +
    // " lon=" + std::to_string(static_cast<float>(_pvt.lon)  * 1e-7f) +
    // " hMSL="+ std::to_string(static_cast<float>(_pvt.hMSL) * 1e-3f) + "m");
}