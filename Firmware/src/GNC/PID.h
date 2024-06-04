#include <Eigen/Dense>
#include "/librrc/Remote/nrcremoteactuatorbase.h"

class PID : public NRCRemoteActuatorBase<PID>
{
    public:
        PID();

        Eigen::Matrix<float,1, 4> outputMatrix();
    private:
         Eigen::Matrix<float,4, 6> K_p;




}