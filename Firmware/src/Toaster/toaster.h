#pragma once
/**
 * @file nrcgreg.h
 * @author Martin England
 * @author Andrei Paduraru (ap2621@ic.ac.uk)
 * @brief The greg class is responsible for all control related to the E-Reg.
 * @version 0.1
 * @date 2024-09-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <librrc/Remote/nrcremoteactuatorbase.h>
#include <librrc/Remote/nrcremoteservo.h>
#include <librrc/Remote/nrcremoteptap.h>
#include <Config/services_config.h>


#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>
#include <libriccore/fsm/statemachine.h>
#include <libriccore/riccorelogging.h>

#include "types.h"

// template <RicCoreLoggingConfig::LOGGERS LOGGING_TARGET = RicCoreLoggingConfig::LOGGERS::SYS>
class Toaster : public NRCRemoteActuatorBase<Toaster>
{
public:
    Toaster(RnpNetworkManager &networkmanager):
        NRCRemoteActuatorBase(networkmanager),
        m_networkmanager(networkmanager) {};

    void setup();
    void update();

protected:
    // Networking
    RnpNetworkManager &m_networkmanager;
    friend class NRCRemoteActuatorBase;
    friend class NRCRemoteBase;

    // Remote actuator implementations
    void arm_base(int32_t arg);
    void disarm_base(int32_t arg);
    void execute_base(int32_t arg);

    // FSM related stuff
    Types::TOASTER_TYPES::StateMachine_t m_ToasterMachine;
    Types::TOASTER_TYPES::SystemStatus_t m_ToasterStatus;
};