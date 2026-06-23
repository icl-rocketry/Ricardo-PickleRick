#pragma once

#include <Arduino.h>
#include <Eigen/Dense>

#include <librrc/Remote/nrcremotecontrollerbase.h>
#include <librnp/rnp_networkmanager.h>
#include <librrc/Helpers/nvsstore.h>

#include "Config/services_config.h"
#include "Config/timing_config.h"
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
        void setBatteryVoltage(float batt_V, bool fresh);
        void setPositionTarget(const Eigen::Vector3f& position,
                               const Eigen::Vector3f& velocity = Eigen::Vector3f::Zero(),
                               const Eigen::Vector3f& acceleration = Eigen::Vector3f::Zero());
        void setPositionControlEnabled(bool enabled);
        void setManualOutput(const Eigen::Vector4f& output, bool actuate);
        unsigned long getStartTime() { return m_controller_start_time; };
        float getBatteryVoltage() const { return m_batt_V; }
        float getVoltageScale() const { return m_voltage_scale; }
        Eigen::Vector4f getOutputValues() const { return m_output; }
        float getCommandedThrustTop() const { return m_output(2); }
        float getCommandedThrustBottom() const { return m_output(3); }
        float getFxCmd() const { return m_pd.getFxCmd(); }
        float getFxCmdOuter() const { return m_pd.getFxCmdOuter(); }
        bool getPositionControlEnabled() const { return m_pd.getPositionControlEnabled(); }
        Eigen::Vector3f getPositionError() { return m_pd.getPositionError(); }
        Eigen::Vector3f getVelocityError() { return m_pd.getVelocityError(); }
        void update(Eigen::Quaterniond q, 
                    Eigen::Vector3f angular_rates, 
                    Eigen::Vector3f position, 
                    Eigen::Vector3f velocity,
                    bool actuate);
        void updateThrottleProfileTest(bool actuate);
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
        unsigned long m_previousSampleTime = 0;
        unsigned long m_actuationDelta = TimingConfig::Controller::ACTUATION_DELTA_MS;
        unsigned long m_controller_start_time;
        unsigned long m_throttle_profile_step_start_time = 0;
        uint8_t m_throttle_profile_step = 0;

        Eigen::Matrix<float,1, 7> m_input;
        Eigen::Vector4f m_output;

        // Eigen::Matrix<float,1, 7> m_setpoint; 

        float m_batt_V = 16.8f;
        bool m_batt_fresh = false;
        float m_voltage_scale = 1.0f;
                
        friend class NRCRemoteBase<GNCController>;
        friend class NRCRemoteControllerBase<GNCController>;
}; 
