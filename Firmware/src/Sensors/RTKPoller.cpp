#include "RTKPoller.h"

void RTKPoller::setup()
{
    // x_input = 0.0f;
    // y_input = 0.0f;
    // z_input = 0.0f;
    // u_input = 0.0f;
    // v_input = 0.0f;
    // w_input = 0.0f;
    m_homeSet = false;
}

void RTKPoller::update(SensorStructs::RTK_t &data)
{
    data.x = x_input;
    data.y = y_input;
    data.z = z_input;
    data.u = u_input;
    data.v = v_input;
    data.w = w_input;
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

    rtkdata.deserializeBody(serializedData);
    x_input = rtkdata.x_input;
    y_input = rtkdata.y_input;
    z_input = rtkdata.z_input;
    u_input = rtkdata.u_input;
    v_input = rtkdata.v_input;
    w_input = rtkdata.w_input;

    Serial.println(rtkdata.x_input);
}
