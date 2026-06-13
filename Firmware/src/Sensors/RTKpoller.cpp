#include "Sensors/RTKpoller.h"
#include "Config/timing_config.h"

void RTKPoller::setup()
{
    x_input = 0.0f;
    y_input = 0.0f;
    z_input = 0.0f;
    u_input = 0.0f;
    v_input = 0.0f;
    w_input = 0.0f;
    fix_quality = 0;
    wifi_connected = false;
    gnss_time_of_day_ms = 0;
    m_homeSet = false;
    m_valid = false;
    m_timestamp_us = 0;
}

void RTKPoller::update(SensorStructs::RTK_t &data)
{
    const uint32_t now_us = micros();
    const Eigen::Vector3f position = getPosition();
    data.x = position.x();
    data.y = position.y();
    data.z = position.z();
    data.u = u_input;
    data.v = v_input;
    data.w = w_input;
    data.fix_quality = fix_quality;
    data.wifi_connected = wifi_connected;
    data.valid = m_homeSet && hasFreshMeasurement(now_us);
    data.gnss_time_of_day_ms = gnss_time_of_day_ms;
    data.timestamp_us = m_timestamp_us;
}

bool RTKPoller::hasMeasurement() const
{
    return hasFreshMeasurement(micros());
}

bool RTKPoller::hasFix() const
{
    return hasMeasurement() && fix_quality != 0;
}

bool RTKPoller::hasFreshMeasurement(const uint32_t now_us) const
{
    return m_valid &&
           m_timestamp_us != 0 &&
           now_us - m_timestamp_us <= TimingConfig::EKF::RTK_CORRECTION_MAX_AGE_US;
}

std::function<void(packetptr_t)> RTKPoller::getThisNetworkCallback()
{
    return [this](packetptr_t packetptr)
    { handlecommand(std::move(packetptr)); };

};

Eigen::Vector3f RTKPoller::getPosition() {
    if (m_homeSet) {
        return Eigen::Vector3f(x_input-x_home, y_input-y_home, z_input-z_home);
    } else {
        return Eigen::Vector3f(x_input, y_input, z_input);
    };
}

Eigen::Vector3f RTKPoller::getPositionRaw() {
    return Eigen::Vector3f(x_input, y_input, z_input);
}

void RTKPoller::handlecommand(packetptr_t packetptr)
{
    std::vector<uint8_t> serializedData = packetptr->getBody();

    // If uncommented, outputs the RNP header fields for each RTK callback packet:
    // source/destination addresses, source/destination services, packet type,
    // uid, body length, expected RTK destination service, and expected body size.
    // Serial.printf(
    //     "RTK DEBUG callback packet: src=%u dst=%u src_service=%u dst_service=%u type=%u uid=%u len=%u expected_dst_service=%u expected_size=%u\n",
    //     packetptr->header.source,
    //     packetptr->header.destination,
    //     packetptr->header.source_service,
    //     packetptr->header.destination_service,
    //     packetptr->header.type,
    //     packetptr->header.uid,
    //     packetptr->header.packet_len,
    //     static_cast<uint8_t>(Services::ID::RTK),
    //     static_cast<unsigned>(RTKPacket::size())
    // );

    // If uncommented, outputs the raw RTK packet body bytes in hexadecimal.
    // Serial.print("RTK DEBUG body bytes:");
    // for (uint8_t byte : serializedData)
    // {
    //     Serial.printf(" %02X", byte);
    // }
    // Serial.println();

    // If uncommented, warns when the packet source is not the expected Chad
    // servo or prop RTK source address.
    // if (packetptr->header.source != 102 && packetptr->header.source != 103)
    // {
    //     Serial.printf(
    //         "RTK DEBUG warning: packet source %u is not an expected Chad address (%u or %u)\n",
    //         packetptr->header.source,
    //         102,
    //         103
    //     );
    // }

    // If uncommented, warns when the packet destination service is not the RTK
    // service id.
    // if (packetptr->header.destination_service != static_cast<uint8_t>(Services::ID::RTK))
    // {
    //     Serial.printf(
    //         "RTK DEBUG warning: destination service %u is not RTK service %u\n",
    //         packetptr->header.destination_service,
    //         static_cast<uint8_t>(Services::ID::RTK)
    //     );
    // }

    if (serializedData.size() != RTKPacket::size()) {
        // If uncommented, outputs the received RTK body size and expected RTK
        // body size before dropping a wrong-sized packet.
        Serial.printf(
            "RTK DEBUG drop: wrong body size got=%u expected=%u\n",
            static_cast<unsigned>(serializedData.size()),
            static_cast<unsigned>(RTKPacket::size())
        );
        Serial.printf(
            "RTK DEBUG packet vector=%u header_size=%u header.packet_len=%u body=%u expected=%u\n",
            static_cast<unsigned>(packetptr->packet.size()),
            static_cast<unsigned>(packetptr->header.size()),
            static_cast<unsigned>(packetptr->header.packet_len),
            static_cast<unsigned>(serializedData.size()),
            static_cast<unsigned>(RTKPacket::size())
        );
        return;
    }

    rtkdata.deserializeBody(serializedData);
    x_input = rtkdata.x_input;
    y_input = rtkdata.y_input;
    z_input = rtkdata.z_input;
    u_input = rtkdata.u_input;
    v_input = rtkdata.v_input;
    w_input = rtkdata.w_input;
    fix_quality = rtkdata.fix_quality;
    wifi_connected = rtkdata.wifi_connected != 0;
    gnss_time_of_day_ms = rtkdata.gnss_time_of_day_ms;
    m_timestamp_us = micros();
    m_valid = true;

    // If uncommented, outputs the decoded RTK position, velocity, fix quality,
    // validity flag, and local timestamp captured after decoding.
    // Serial.printf(
    //     "RTK DEBUG decoded: x=%.3f y=%.3f z=%.3f u=%.3f v=%.3f w=%.3f fix=%u valid=%u timestamp_us=%lu\n",
    //     x_input,
    //     y_input,
    //     z_input,
    //     u_input,
    //     v_input,
    //     w_input,
    //     fix_quality,
    //     m_valid ? 1 : 0,
    //     static_cast<unsigned long>(m_timestamp_us)
    // );
}
