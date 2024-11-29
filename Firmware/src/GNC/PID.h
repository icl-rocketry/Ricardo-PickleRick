#include <Eigen/Dense>
#include "librrc/Remote/nrcremoteactuatorbase.h"
#include <librnp/rnp_networkmanager.h>
#include <librrc/Helpers/nvsstore.h>
#include "GNC/PIDcalibrationpacket.h"


class PID : public NRCRemoteActuatorBase<PID>
{
    public:
        PID(RnpNetworkManager &networkmanager):
        NRCRemoteActuatorBase(networkmanager) 
        {};
        
        Eigen::Matrix<float,1, 6> outputMatrix(Eigen::Matrix<float,4, 6> inputMatrix);
        void Setup();
        void save_pid_gains();

    private:
        Eigen::Matrix<float,4, 6> K_p;
        void createTestK_p();
}