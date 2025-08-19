#pragma once
#include <librrc/Remote/nrcremotebase.h>
#include "./packets/rtkpacket.h"
#include "Arduino.h"
#include <librnp/rnp_interface.h>
#include "Sensors/sensorStructs.h"

class RTKPoller
{
public:
    RTKPoller() {};
    void setup();
    void update(SensorStructs::RTK_t &data);
    void setHome(Eigen::Vector3f position) {
        x_home = position.x();
        y_home = position.y();
        z_home = position.z();
        m_homeSet = true;
    };
    std::function<void(packetptr_t)> getThisNetworkCallback();
    Eigen::Vector3f getPosition();
    Eigen::Vector3f getPositionRaw();
    float x_input = 11.0f;
    float y_input = 12.0f;
    float z_input = 13.0f;
    float u_input = 14.0f;
    float v_input = 15.0f;
    float w_input = 16.0f;
    float x_home = 0.0f;
    float y_home = 0.0f;
    float z_home = 0.0f;
    bool m_homeSet;
private:

    unsigned long m_prev_timestamp = 0;

    void handlecommand(packetptr_t packetptr);

    RTKPacket rtkdata;
};