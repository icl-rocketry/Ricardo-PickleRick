#include "Sensors/RTKpoller.h"

void RTKPoller::setup()
{
    x_input = 0.0f;
    y_input = 0.0f;
    z_input = 0.0f;
    u_input = 0.0f;
    v_input = 0.0f;
    w_input = 0.0f;
    fix_quality = 0;
    m_homeSet = false;
    m_valid = false;
    m_timestamp_us = 0;
}

void RTKPoller::update(SensorStructs::RTK_t &data)
{
    const Eigen::Vector3f position = getPosition();
    data.x = position.x();
    data.y = position.y();
    data.z = position.z();
    data.u = u_input;
    data.v = v_input;
    data.w = w_input;
    data.fix_quality = fix_quality;
    data.valid = m_valid;
    data.timestamp_us = m_timestamp_us;
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
    if (serializedData.size() != RTKPacket::size()) {
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
    m_timestamp_us = micros();
    m_valid = fix_quality != 0;
}
