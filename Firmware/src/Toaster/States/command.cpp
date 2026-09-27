#include "command.h"

#include <libriccore/fsm/state.h>

#include "Config/commands_config.h"
#include "Config/general_config.h"
#include "Toaster/States/zero.h"
#include "Toaster/types.h"

#include "system.h"

Command::Command(System& system, Types::TOASTER_TYPES::SystemStatus_t& status):
    State(TOASTER_FLAGS::STATE_COMMAND, status),
    m_system(system),
    m_status(status),
    m_pid(GeneralConfig::PIDRollKP, GeneralConfig::PIDRollKI, GeneralConfig::PIDRollKD,
          GeneralConfig::PIDRollILimit, GeneralConfig::PIDRollDLimit) {};

void Command::initialize(){
    State::initialize();

    m_pid.setErrorFunction(std::bind(Command::angleErrorFunction, std::placeholders::_1, std::placeholders::_2));
    m_pid.setDerivativeFunction(std::bind(Command::angleDerivativeFunction, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    m_lastUpdateTimeMs = millis();

    m_system.toaster.stepperCommand(Toaster::StepperCommandPayload::HOLD);
    m_system.toaster.stepperEnable();

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered Command state.");
};

Types::TOASTER_TYPES::State_ptr_t Command::update(){
    // Get current state from estimator
    const SensorStructs::state_t& measurement = m_system.estimator.getData();

    const uint64_t timeMs = millis();
    const uint64_t dtMs = timeMs - m_lastUpdateTimeMs;
    const double dt = static_cast<double>(dtMs) / 1000.0;

    m_lastUpdateTimeMs = timeMs;

    const double roll = measurement.rocketEulerAngles[0];
    const double targetRoll = m_targetRollGenerator.getRollSetpoint(timeMs - this->time_entered_state);

    // Run PID update to get torque (Nm)
    const double torque = m_pid.update(targetRoll, roll, dt);

    // Send command to the actuators at designated time delta.
    if (timeMs - m_lastCommandTimeMs >= GeneralConfig::CommandDeltaMs) {
        m_lastCommandTimeMs = timeMs;
        m_system.toaster.commandTorque(torque);
    }

    m_system.toaster.updatePIDLog(m_pid.getLog());

    return nullptr;
};

void Command::exit(){
    Types::TOASTER_TYPES::State_t::exit();
}

double Command::angleErrorFunction(double target, double measurement) {
    return std::remainder(target - measurement, 2.0 * M_PI);
}

double Command::angleDerivativeFunction(const double measurement, const double prevMeasurement, const double dt) {
    return std::remainder(measurement - prevMeasurement, 2.0 * M_PI) / dt;
}