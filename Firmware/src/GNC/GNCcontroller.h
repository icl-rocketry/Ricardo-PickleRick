#pragma once

#include <Eigen/Dense>
#include "librrc/Remote/nrcremotecontrollerbase.h"
#include <librnp/rnp_networkmanager.h>

#include <librrc/Helpers/nvsstore.h>
#include "GNC/PIDCalibrationPacket.h"
#include "GNC/PIDTelemetryPacket.h"
#include "Config/services_config.h"
#include <Arduino.h>
#include "GNC/PID.h"

class GNCcontroller : public NRCRemoteControllerBase<GNCcontroller>
{
    public: 
        GNCcontroller(std::string name, Services::ID serviceID, RnpNetworkManager& networkmanager, PID& pid1 , PID& pid2):
            NRCRemoteControllerBase(name, networkmanager), 
            m_networkmanager(networkmanager),
            m_serviceID(static_cast<uint8_t>(serviceID)),
            pid1(pid1), 
            pid2(pid2) 
            {};

        void setup(); 
        void start();
        void update(Eigen::Matrix<float,1, 6> currentInput); 
        void stop();

        
    private: 
        
        void sendArmingCommands(); 
        void sendDisarmingCommands();
        void sendActuationCommands(Eigen::Matrix<float,1, 4> actuation_values);
        void armServos();
        void disarmServos();
        void changeServoAngle(int servo, int angle);
        void armProps();
        void disarmProps();
        void changePropPower(int prop, int power);

        RnpNetworkManager &m_networkmanager;
        uint8_t m_serviceID;
        unsigned long m_previousSampleTime;
        unsigned long m_actuationDelta = 10; // 0.1 second

        Eigen::Matrix<float,1, 6> input_first;
        Eigen::Matrix<float,1, 4> output_first;
        Eigen::Matrix<float,1, 6> input_second; 
        Eigen::Matrix<float,1, 4> output_second;

        Eigen::Matrix<float,1, 6> setpoint_first; 
        Eigen::Matrix<float,1, 6> setpoint_second; 

        void telemetry_impl(packetptr_t packetptr);

        PID& pid1; 
        PID& pid2; 
                
        friend class NRCRemoteBase<GNCcontroller>;
        friend class NRCRemoteControllerBase<GNCcontroller>;
}; 