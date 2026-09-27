#include "deploy.h"

#include <libriccore/fsm/state.h>

#include "Config/commands_config.h"
#include "Config/general_config.h"

#include "Toaster/types.h"
#include "Toaster/States/command.h"

#include "system.h"

Deploy::Deploy(System& system, Types::TOASTER_TYPES::SystemStatus_t& status):
    State(TOASTER_FLAGS::STATE_DEPLOY, status),
    m_system(system),
    m_status(status) {};

void Deploy::initialize(){
    State::initialize();

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered Deploy state.");

    m_timeEnterStateMs = millis();

    // Ensure stepper motor enabled
    m_system.toaster.stepperEnable();
};

Types::TOASTER_TYPES::State_ptr_t Deploy::update(){
    // Wait for the upper endstop
    if (m_system.toaster.upperEndstopReached()) {
        return std::make_unique<Command>(m_system, m_status);
    }

    const uint64_t timeMs = millis();

    // Wait for the deploy delay
    if (timeMs - m_timeEnterStateMs < GeneralConfig::DeployDelayMs) {
        return nullptr;
    }

    // Send grid fin extend command
    if (timeMs - m_lastCommandTimeMs >= GeneralConfig::CommandDeltaMs) {
        m_system.toaster.stepperCommand(Toaster::StepperCommandPayload::EXTEND);
    }

    return nullptr;
};

void Deploy::exit(){
    m_system.toaster.stepperCommand(Toaster::StepperCommandPayload::HOLD);
    Types::TOASTER_TYPES::State_t::exit();
};