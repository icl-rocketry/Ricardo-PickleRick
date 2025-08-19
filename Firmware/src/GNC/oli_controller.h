#pragma once

#include <Eigen/Dense>
#include "librrc/Remote/nrcremotecontrollerbase.h"
#include <librnp/rnp_networkmanager.h>

#include <librrc/Helpers/nvsstore.h>
// #include "GNC/oli_controllerCalibrationPacket.h"
// #include "GNC/oli_controllerTelemetryPacket.h"
#include "Config/services_config.h"
#include <Arduino.h>
 


class Oli_controller : public NRCRemoteControllerBase<Oli_controller>
{
    public:
        Oli_controller(std::string name, Services::ID serviceID, RnpNetworkManager& networkmanager):
            NRCRemoteControllerBase(name, networkmanager), 
            m_networkmanager(networkmanager),
            m_serviceID(static_cast<uint8_t>(serviceID)) 
            {};

        void setup(Eigen::Matrix<float,1, 12> m_personal_setpoint);
        void update(Eigen::Matrix<float,1, 12> currentPosition);
        void reset();
        void check_gains();
        Eigen::Matrix<float,1, 4> getOutputValues(); 
    
    private:

        void updateOutputValues(Eigen::Matrix<float,1, 12> currentPosition);
        void updateErrors(Eigen::Matrix<float,1, 6> currentPosition);

        RnpNetworkManager &m_networkmanager;
        uint8_t m_serviceID;
        float m_timestep; 
        unsigned long m_previousSampleTime;

        Eigen::Matrix<float,6, 4> m_K_p;
        Eigen::Matrix<float,6, 4> m_K_i;
        Eigen::Matrix<float,6, 4> m_K_d;

        // OUTER LOOP TRANSLATION GAINS
        Eigen::Matrix<float,1, 3> m_kPos1; // k₁ for x, y, z
        Eigen::Matrix<float,1, 3> m_kPos2; // k₂ for x, y, z
        Eigen::Matrix<float,1, 3> m_lambdaPosOuter; // λ for x, y, z
        float m_etaPosOuter; // η for x, y, z
        Eigen::Matrix<float,1, 3> m_psiPosOuter; // ψ for x, y, z
        // INNER LOOP TRANSLATION GAINS
        float m_kPos3; // Fz inner loop 1st gain
        float m_kPos4; // Fz inner loop 2nd gain 
        float m_lambdaPosInner; // λ for Fz inner loop
        float m_etaPosInner; // η for Fz inner loop
        float m_psiPosInner; // ψ for Fz inner loop
        //ATTITUDE GAINS
        float m_kAtt1; // k₁ for attitude control
        float m_kAtt2; // k₂ for attitude control
        float m_lambdaAtt; // λ for attitude control
        float m_etaAtt; // η for attitude control
        float m_psiAtt; // ψ for attitude control
        /* ---- rocket physical parameters ------------------------------------ */  
        float           m_mass;       // kg
        Eigen::Matrix3f m_J;
        float           m_dtCtrl;     // control period [s]
        float           m_rEngZ;      // distance nozzle ↔ CoM [m]

        /* ---- actuator lag parameters --------------------------------------- */
        float m_tauAct;   // s  (time constant)
        float m_kAct;   // s⁻¹ (error gain)
        Eigen::Vector3f m_u_act; // actuator output [N] (Fx, Fy, Fz)
        Eigen::Vector3f m_eta_prev; // previous eta for attitude control
        Eigen::Vector3f m_etaDot_prev; // previous eta_dot for attitude control
        /* ---- first-order derivative low-pass ------------------------------- */
        float m_dfilterA;  // (≈ exp(-dt/τ))

        Eigen::Matrix<float,1, 12> m_setpoint;
        Eigen::Matrix<float,1, 12> m_error;

        Eigen::Matrix<float,1, 4> m_output_values; 

        void calibrate_impl(packetptr_t packetptr);
        void telemetry_impl(packetptr_t packetptr);
        
        friend class NRCRemoteBase<Oli_controller>;
        friend class NRCRemoteControllerBase<Oli_controller>;

};