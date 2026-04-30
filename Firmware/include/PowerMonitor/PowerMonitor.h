#pragma once

#include <Arduino.h>
#include "librrc/Remote/nrcremotebase.h"

struct PowerMonitorData
{
    uint16_t batt_mV = 16800;
    bool fresh = false;
    uint32_t last_update_ms = 0;
};

class PowerMonitor : public NRCRemoteBase<PowerMonitor>
{
public:
    PowerMonitor(const char* name, uint8_t service_id, RnpNetworkManager& networkmanager);

    void setup();
    void update();
    
    void networkCallback(packetptr_t packetptr);

    const PowerMonitorData& getData() const;

    float getBatteryVoltage() const;
    uint16_t getBatteryMilliVolts() const;
    bool fresh() const;


private:
    void requestPdbTelem();
    

    PowerMonitorData m_data;

    uint32_t m_last_request_ms = 0;
    uint8_t m_service_id;

    static constexpr uint8_t PDB_NODE_ID = 20;
    static constexpr uint8_t LMQ_COMMAND_SERVICE = 2;
    static constexpr uint8_t LMQ_TELEM_COMMAND = 4;

    static constexpr uint32_t REQUEST_PERIOD_MS = 1000;
    static constexpr uint32_t FRESH_TIMEOUT_MS = 1000;
};