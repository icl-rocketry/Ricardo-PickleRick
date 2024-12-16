#include <Eigen/Dense>
#include "librrc/Remote/nrcremoteactuatorbase.h"
#include <librnp/rnp_networkmanager.h>

#include <librrc/Helpers/nvsstore.h>
#include "GNC/pidcalibrationpacket.h"

#include <Arduino.h>



class PID : public NRCRemoteActuatorBase<PID>
{
    public:
        PID(RnpNetworkManager &networkmanager):
        NRCRemoteActuatorBase(networkmanager), m_networkmanager(networkmanager) 
        {};

        void updateActuationValues();
        void setup();
        void update(Eigen::Matrix<float,1, 6> currentPosition);
        void check_gains();
        void sendActuationCommands();
    private:
        RnpNetworkManager &m_networkmanager;
        
        Eigen::Matrix<float,6, 4> K_p;
        Eigen::Matrix<float,6, 4> K_i;
        Eigen::Matrix<float,6, 4> K_d;

        Eigen::Matrix<float,1, 6> setpoint;
        float timestep; 
        unsigned long previousSampleTime;
        unsigned long actuationDelta = 100; // 1 second
        Eigen::Matrix<float,1, 6> error;
        Eigen::Matrix<float,1, 6> integral_error_riemman; 
        Eigen::Matrix<float,1, 6> integral_error_trapezoid; 
        Eigen::Matrix<float,1, 6> previous_error; //update this change later
        Eigen::Matrix<float,1, 6> derivative_error; 
        Eigen::Matrix<float,1, 4> actuation_values; 

        void calibrate_impl(packetptr_t packetptr);
        void createTestK_p();
        void createTestK_i();
        void createTestK_d();
        void updateErrors(Eigen::Matrix<float,1, 6> currentPosition);
        void armServos();
        void changeServoAngle(int servo, int angle);
        void changePropPower(int prop, int power) ;

};