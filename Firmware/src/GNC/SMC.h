#include <Eigen/Dense>
#include "librrc/Remote/nrcremotecontrollerbase.h"
#include <librnp/rnp_networkmanager.h>

#include <librrc/Helpers/nvsstore.h>
#include "GNC/SMCCalibrationPacket.h"
#include "GNC/SMCTelemetryPacket.h"
#include "Config/services_config.h"
#include <Arduino.h>



class LQR_SMC : public NRCRemoteControllerBase<LQR_SMC>
{
    public:
        LQR_SMC(std::string name, Services::ID serviceID, RnpNetworkManager &networkmanager):
            NRCRemoteControllerBase(name, networkmanager), 
            m_networkmanager(networkmanager),
            m_serviceID(static_cast<uint8_t>(serviceID)) {};

        void updateActuationValues();
        void setup();
        void update(Eigen::Matrix<float,1, 6> currentPosition);
        void check_gains();
        void updateSMC();
        float getThrust1(){return m_actuation_values(0,2);};
        float getThrust2(){return m_actuation_values(0,3);};

    private:
        RnpNetworkManager &m_networkmanager;
        uint8_t m_serviceID;
        Eigen::Matrix<float,6, 4> m_K_p;
        Eigen::Matrix<float,6, 4> m_K_i;

        Eigen::Matrix<float,1, 6> m_setpoint;
        float m_timestep; 
        unsigned long m_previousSampleTime;
        unsigned long m_actuationDelta = 100; // 1 second
        Eigen::Matrix<float,1, 6> m_error;
        Eigen::Matrix<float,1, 6> m_integral_error_riemman; 
        Eigen::Matrix<float,1, 6> m_integral_error_trapezoid; 
        Eigen::Matrix<float,1, 6> m_previous_error; //update this change later
        Eigen::Matrix<float,1, 4> m_actuation_values;
        Eigen::Matrix<float, 1, 4> m_smc_actuation_values;

        void createTestK_p();
        void createTestK_i();
        void createTestLambda();
        void createTestEta();
        void createTestPsi();
        void updateErrors(Eigen::Matrix<float,1, 6> currentPosition);

        void calibrate_impl(packetptr_t packetptr);
        void telemetry_impl(packetptr_t packetptr);

        Eigen::Matrix<float,1, 3> lambda_smc;
        Egien::Matrix<float,1, 3> eta_smc;
        Eigen::Matrix<float,1, 3> psi_smc;
        
        friend class NRCRemoteBase<SMC>;
        friend class NRCRemoteControllerBase<SMC>;

};
