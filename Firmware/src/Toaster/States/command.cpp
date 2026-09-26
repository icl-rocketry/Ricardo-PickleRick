#include "command.h"

#include <libriccore/fsm/state.h>

#include "Config/commands_config.h"
#include "Config/general_config.h"
#include "Toaster/types.h"

#include "system.h"

Command::Command(System& system, Types::TOASTER_TYPES::SystemStatus_t& status):
    State(TOASTER_FLAGS::STATE_COMMAND, status),
    m_system(system),
    m_pid(GeneralConfig::PIDRollKP, GeneralConfig::PIDRollKI, GeneralConfig::PIDRollKD,
          GeneralConfig::PIDRollILimit, GeneralConfig::PIDRollDLimit) {};

void Command::initialize(){
    State::initialize();

    m_pid.setup([](double measurement, double target) {
        return target - measurement;
    });

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered Command state.");
};

Types::TOASTER_TYPES::State_ptr_t Command::update(){
    const uint64_t currentMs = millis();

    // Get current state from estimator
    const SensorStructs::state_t& measurement = m_system.estimator.getData();
    const double roll = measurement.rocketEulerAngles[0];

    const double targetRoll = m_targetRollGenerator.getRollSetpoint(currentMs);

    // Ensure wrapped roll values never exceed 2 rolls
    if (std::abs(roll) > 4 * M_PI || std::abs) {}

    // Run PID update


    // Send command to the actuators at designated time delta.
    // m_system.toaster.actuatorCommand(m_actuatorCommand);
    return nullptr;
};

void Command::exit(){
    Types::TOASTER_TYPES::State_t::exit();
};