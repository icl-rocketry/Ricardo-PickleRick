#pragma once

#include <Eigen/Dense>
#include "librrc/Remote/nrcremotecontrollerbase.h"
#include <librnp/rnp_networkmanager.h>

#include <librrc/Helpers/nvsstore.h>
#include "GNC/PIDCalibrationPacket.h"
#include "GNC/PIDTelemetryPacket.h"
#include "Config/services_config.h"
#include <Arduino.h>
 


class PID : public NRCRemoteControllerBase<PID>
{
    public:
        PID(std::string name, Services::ID serviceID, RnpNetworkManager& networkmanager):
            NRCRemoteControllerBase(name, networkmanager), 
            m_networkmanager(networkmanager),
            m_serviceID(static_cast<uint8_t>(serviceID)) 
            {};

        void setup(Eigen::Matrix<float,1, 6> m_personal_setpoint);
        void update(Eigen::Matrix<float,1, 6> currentPosition);
        void reset();
        void check_gains();
        Eigen::Matrix<float,1, 4> getOutputValues(); 
    
    private:

        void updateOutputValues();
        void createTestK_p();
        void createTestK_i();
        void createTestK_d();
        void updateErrors(Eigen::Matrix<float,1, 6> currentPosition);

        RnpNetworkManager &m_networkmanager;
        uint8_t m_serviceID;
        float m_timestep; 
        unsigned long m_previousSampleTime;

        Eigen::Matrix<float,6, 4> m_K_p;
        Eigen::Matrix<float,6, 4> m_K_i;
        Eigen::Matrix<float,6, 4> m_K_d;

        Eigen::Matrix<float,1, 6> m_setpoint;
        Eigen::Matrix<float,1, 6> m_error;
        Eigen::Matrix<float,1, 6> m_integral_error_riemman; 
        Eigen::Matrix<float,1, 6> m_integral_error_trapezoid; 
        Eigen::Matrix<float,1, 6> m_previous_error; //update this change later
        Eigen::Matrix<float,1, 6> m_derivative_error; 
        Eigen::Matrix<float,1, 6> m_sum_error; //

        Eigen::Matrix<float,1, 4> m_output_values; 

        void calibrate_impl(packetptr_t packetptr);
        void telemetry_impl(packetptr_t packetptr);
        
        friend class NRCRemoteBase<PID>;
        friend class NRCRemoteControllerBase<PID>;

};