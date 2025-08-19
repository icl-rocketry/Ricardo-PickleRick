#pragma once

#include <Eigen/Dense>
#include "librrc/Remote/nrcremotecontrollerbase.h"
#include <librnp/rnp_networkmanager.h>

#include <librrc/Helpers/nvsstore.h>
#include "GNC/PIDCalibrationPacket.h"
#include "GNC/ControllerTelemetryPacket.h"
#include "Config/services_config.h"
#include <Arduino.h>
#include "GNC/oli_controller.h"

class GNCcontroller : public NRCRemoteControllerBase<GNCcontroller>
{
    public: 
        GNCcontroller(std::string name, Services::ID serviceID, RnpNetworkManager& networkmanager, Oli_controller& oli_controller):
            NRCRemoteControllerBase(name, networkmanager), 
            m_networkmanager(networkmanager),
            m_serviceID(static_cast<uint8_t>(serviceID)), 
            oli_controller(oli_controller) 
            {};

        void setup(); 
        void start();
        void update(Eigen::Matrix<float,1, 13> currentInput); 
        void stop();

        
    private: 
        
        void sendArmingCommands(); 
        void sendDisarmingCommands();
        void sendActuationCommands(Eigen::Matrix<float,1, 4> actuation_values,float ramp_up_value);
        void armServos();
        void disarmServos();
        void changeServoAngle(int servo, float angle);
        void armProps();
        void disarmProps();
        void changePropPower(int prop, int power);

        RnpNetworkManager &m_networkmanager;
        uint8_t m_serviceID;
        unsigned long m_previousSampleTime;
        unsigned long m_actuationDelta = 5; // 0.005 seconds (200 Hz)

        Eigen::Matrix<float,1, 12> input_first;
        Eigen::Matrix<float,1, 4> output_first;
        Eigen::Matrix<float,1, 12> input_second; 
        Eigen::Matrix<float,1, 4> output_second;

        Eigen::Matrix<float,1, 12> setpoint_first; 
        Eigen::Matrix<float,1, 12> setpoint_second; 

        void telemetry_impl(packetptr_t packetptr);
        Oli_controller& oli_controller; 
                
        friend class NRCRemoteBase<GNCcontroller>;
        friend class NRCRemoteControllerBase<GNCcontroller>;
}; 