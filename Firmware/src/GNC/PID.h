#include <Eigen/Dense>
#include "librrc/Remote/nrcremoteactuatorbase.h"
#include <librnp/rnp_networkmanager.h>


class PID : public NRCRemoteActuatorBase<PID>
{
    public:
        PID(RnpNetworkManager &networkmanager):
        NRCRemoteActuatorBase(networkmanager) 
        {};
        Eigen::Matrix<float,1, 4> outputMatrix(Eigen::Matrix<float,1, 6> inputMatrix);
        void Setup();
    private:
        Eigen::Matrix<float,6, 4> K_p;
        void createTestK_p();
};