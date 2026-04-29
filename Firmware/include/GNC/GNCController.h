#pragma once

#include <Arduino.h>
#include <Eigen/Dense>

#include <librrc/Remote/nrcremotecontrollerbase.h>
#include <librnp/rnp_networkmanager.h>
#include <librrc/Helpers/nvsstore.h>

#include "Config/services_config.h"
#include "GNC/ControllerTelemetryPacket.h"
#include "GNC/PDController.h"

class GNCController : public NRCRemoteControllerBase<GNCController>
{
    public: 
        GNCController(std::string name, Services::ID serviceID, RnpNetworkManager& networkmanager):
            NRCRemoteControllerBase(name, networkmanager), 
            m_networkmanager(networkmanager),
            m_serviceID(static_cast<uint8_t>(serviceID))
            {};

        void setup(); 
        void start();
        unsigned long getStartTime() { return m_controller_start_time; };
        void update(Eigen::Matrix<float,1, 7> currentInput, bool actuate); 
        void stop();

        
    private: 
        
        void sendArmingCommands(); 
        void sendDisarmingCommands();
        void sendActuationCommands(Eigen::Vector4f actuation_values);
        void armServos();
        void disarmServos();
        void changeServoAngle(int servo, float angle);
        void armProps();
        void disarmProps();
        void changePropPower(int prop, int power);
        void telemetry_impl(packetptr_t packetptr);

        PDController m_pd;
        RnpNetworkManager &m_networkmanager;

        uint8_t m_serviceID;
        unsigned long m_previousSampleTime;
        unsigned long m_actuationDelta = 4; // 0.004 seconds (250 Hz)
        unsigned long m_controller_start_time;

        Eigen::Matrix<float,1, 7> m_input;
        Eigen::Vector4f m_output;

        // Eigen::Matrix<float,1, 7> m_setpoint; 

                
        friend class NRCRemoteBase<GNCController>;
        friend class NRCRemoteControllerBase<GNCController>;
}; 