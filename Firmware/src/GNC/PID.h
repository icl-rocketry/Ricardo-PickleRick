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

        Eigen::Matrix<float,1, 4> updateActuationValues(Eigen::Matrix<float,1, 6> inputMatrix);
        void setup();
        void update(Eigen::Matrix<float,1, 6> currentPosition);
        void check_gains();
        void sendActuationCommands();
    private:
        Eigen::Matrix<float,4, 6> K_p;
        Eigen::Matrix<float,4, 6> K_i;
        Eigen::Matrix<float,4, 6> K_d;

        Eigen::Matrix<float,1, 6> setpoint;
        float timestep; 

        Eigen::Matrix<float,1, 6> error;
        Eigen::Matrix<float,1, 6> integral_error_riemman; 
        Eigen::Matrix<float,1, 6> integral_error_trapezoid; 
        Eigen::Matrix<float,1, 6> previousError; 
        Eigen::Matrix<float,1, 6> derivative_error; 

        void calibrate_impl(packetptr_t packetptr);
        void createTestK_p();
        void updateErrors(Eigen::Matrix<float,1, 6> currentPosition);
        void armServos();
        void changeServoAngle(int servo, int angle);
};