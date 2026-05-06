#include "PowerMonitor/PowerMonitor.h"

#include "Commands/packets/LMQTelempacket.h"
#include <librnp/default_packets/simplecommandpacket.h>
#include "Config/loggerhandler_config.h"
#include "libriccore/riccorelogging.h"

PowerMonitor::PowerMonitor(const char* name,

    uint8_t service_id,

    RnpNetworkManager& networkmanager)

: NRCRemoteBase(name, networkmanager),

m_service_id(service_id)

{}
void PowerMonitor::setup()
{
}

void PowerMonitor::update()
{
    const uint32_t now = millis();

    if (now - m_last_request_ms > REQUEST_PERIOD_MS)
    {
        requestPdbTelem();
        m_last_request_ms = now;
    }
    

    m_data.fresh = (now - m_data.last_update_ms) < FRESH_TIMEOUT_MS;
}

void PowerMonitor::networkCallback(packetptr_t packetptr)
{
    LMQTelemPacket pkt(*packetptr);

    m_data.batt_mV = pkt.battVoltage;
    m_data.last_update_ms = millis();
    m_data.fresh = true;

    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(

    //     "RECEIVED LMQ batt mV: " + std::to_string(m_data.batt_mV)

    // );
}

const PowerMonitorData& PowerMonitor::getData() const
{
    return m_data;
}

float PowerMonitor::getBatteryVoltage() const
{
    return m_data.batt_mV * 0.001f;
}

uint16_t PowerMonitor::getBatteryMilliVolts() const
{
    return m_data.batt_mV;
}

bool PowerMonitor::fresh() const
{
    return m_data.fresh;
}

void PowerMonitor::requestPdbTelem()
{
    SimpleCommandPacket cmd(LMQ_TELEM_COMMAND, 0);

    cmd.header.source = _networkmanager.getAddress();
    cmd.header.source_service = m_service_id;

    cmd.header.destination = PDB_NODE_ID;
    cmd.header.destination_service = LMQ_COMMAND_SERVICE;
    
    _networkmanager.sendPacket(cmd);

    static uint32_t last_log_ms = 0;

    if (millis() - last_log_ms > 5000)

    {

        // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(

        //     "Requested LMQ telemetry"

        // );

        last_log_ms = millis();

    }

}