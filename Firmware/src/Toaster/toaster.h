#pragma once

#include <librrc/Remote/nrcremoteactuatorbase.h>
#include <librrc/Remote/nrcremoteservo.h>
#include <librrc/Remote/nrcremoteptap.h>
#include <Config/services_config.h>


#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>
#include <libriccore/fsm/statemachine.h>
#include <libriccore/riccorelogging.h>

#include "types.h"

class System;

class Toaster : public NRCRemoteActuatorBase<Toaster>
{
public:
    /**
     * @brief Construct a new Toaster object.
     *
     * The Toaster class initialises the enable pin so no need to configure
     * it before / after passing it in.
     *
     * @param networkmanager
     * @param enablePin
     */
    Toaster(System& system, RnpNetworkManager &networkmanager, const int& enablePin, const int& endstopLower, const int& endstopUpper):
        NRCRemoteActuatorBase(networkmanager),
        m_system(system),
        m_networkmanager(networkmanager),
        m_enablePin(enablePin),
        m_endstopLower(endstopLower),
        m_endstopUpper(endstopUpper) {};

    /**
     * @brief Initialise the Toaster control system.
     */
    void setup();

    /**
     * @brief Run update function.
     */
    void update();

    /**
     * @brief External apogee detected.
     */
    void apogeeDetected();

    enum class StepperCommandPayload: uint16_t {
        HOLD = 0,
        EXTEND = 1,
        RETRACT = 2
    };

    // Stepper motor commands
    void stepperCommand(const StepperCommandPayload& command);
    void stepperEnable();
    void stepperDisable();

    // Actuator commands

    /**
     * @brief Actuator command payload.
     *
     * Actuation angles are given in units of 0.1 deg.
     */
    struct ActuatorCommandPayload {
        uint16_t act0Deg;
        uint16_t act1Deg;
        uint16_t act2Deg;
    };

    /// @brief Command the actuators to the zero position.
    void actuatorZero();

    /// @brief Send a command to the actuators.
    /// @param command The target positions.
    void actuatorCommand(const ActuatorCommandPayload& command);

    // Comamnds
    static constexpr uint32_t ACTUATOR_COMMAND_TYPE = 50;
    static constexpr uint32_t STEPPER_COMMAND_TYPE = 51;

    using ActuatorCommandPacket = BasicDataPacket<ActuatorCommandPayload, 0, ACTUATOR_COMMAND_TYPE>;
    using StepperCommandPacket = BasicDataPacket<StepperCommandPayload, 0, ACTUATOR_COMMAND_TYPE>;

    // Endstop read methods
    bool upperEndstopReached();
    bool lowerEndstopReached();

    // Flight dynamics model control

    /**
     * @brief Request a certain torque be commanded from the gridfins.
     */
    void commandTorque(const double torque);

protected:
    // System
    System& m_system;

    // Networking
    RnpNetworkManager& m_networkmanager;
    friend class NRCRemoteActuatorBase;
    friend class NRCRemoteBase;

    // Motor control
    const int m_enablePin;
    const int m_endstopLower;
    const int m_endstopUpper;

    // Remote actuator implementations

    /**
     * @brief Arm the Toaster module.
     *
     * This transitions the Toaster module into armed state, enabling transition into deploy / commanded state.
     *
     * @param arg 0: Transition into armed state.
     * @param arg 1: Transition into armed state and zero the stepper motor on the lower endstop.
     * @param arg 2: Transition directly to command state.
     */
    void arm_base(int32_t arg);

    /**
     * @brief Disarm the Toaster module.
     */
    void disarm_base();

    /**
     * @brief Execute a command in the Toaster module.
     *
     * This command is only implemented for the command state.
     *
     * The argument for this command is the bitwise or'ing of the actuator id
     * and 10x the commanded angle.
     *
     * So for actuator id 2, commanded to 39 degrees would be:
     *      arg = (2 << 16) & 0xffff0000 | (39 * 10) & 0xffff;
     *
     * @param arg act_id << 16 | degrees * 10
     */
    void execute_base(int32_t arg);

    // FSM related stuff
    Types::TOASTER_TYPES::StateMachine_t m_toasterMachine;
    Types::TOASTER_TYPES::SystemStatus_t m_toasterStatus;

    using ToasterCommandData = uint32_t;
    static constexpr uint32_t TOASTER_COMMAND_TYPE = 50;
    using ToasterCommandPacket = BasicDataPacket<ToasterCommandData, 0, TOASTER_COMMAND_TYPE>;
};