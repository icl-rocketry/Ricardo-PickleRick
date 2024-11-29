#include <Eigen/Dense>
#include "librrc/Remote/nrcremoteactuatorbase.h"
#include <librnp/rnp_networkmanager.h>

#include <librrc/Helpers/nvsstore.h>
#include "GNC/PIDcalibrationpacket.h"

#include <Arduino.h>



class PID : public NRCRemoteActuatorBase<PID>
{
    public:
        PID(RnpNetworkManager &networkmanager):
        NRCRemoteActuatorBase(networkmanager) 
        {};

        Eigen::Matrix<float,1, 4> outputMatrix(Eigen::Matrix<float,1, 6> inputMatrix);
        void setup();
        void updateErrors(Eigen::Matrix<float,1, 6> currentPosition);
        void check_gains();

    private:
        Eigen::Matrix<float,4, 6> K_p;
        void createTestK_p();

        Eigen::Matrix<float,1, 6> setpoint; 
        float timestep; 
        Eigen::Matrix<float,1, 6> error;
        Eigen::Matrix<float,1, 6> integral_error_riemman; 
        Eigen::Matrix<float,1, 6> integral_error_trapezoid; 
        Eigen::Matrix<float,1, 6> previousError; 
        void calibrate_impl(packetptr_t packetptr);

};

