#include "zero.h"

#include <libriccore/fsm/state.h>

#include "Toaster/States/armed.h"

#include "Config/commands_config.h"
#include "Config/general_config.h"
#include "Toaster/types.h"
#include "Toaster/toaster.h"

#include "deploy.h"
#include "default.h"

#include "system.h"

Zero::Zero(System& system, Types::TOASTER_TYPES::SystemStatus_t& status):
    State(TOASTER_FLAGS::STATE_ZERO, status),
    m_system(system),
    m_status(status) {};

void Zero::initialize(){
    State::initialize();

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered Zero state.");

    m_system.toaster.stepperEnable();
    m_system.toaster.stepperCommand(Toaster::StepperCommandPayload::RETRACT);

    m_lastCommandTimeMs = millis();
};

Types::TOASTER_TYPES::State_ptr_t Zero::update(){
    // Check endstop first
    if (m_system.toaster.lowerEndstopReached()) {
        // Transition to armed state
        return std::make_unique<Armed>(m_system, m_status);
    }

    const uint64_t timeMs = millis();

    if (timeMs - m_lastCommandTimeMs < GeneralConfig::CommandDeltaMs) {
        return nullptr;
    } else {
        m_system.toaster.stepperCommand(Toaster::StepperCommandPayload::RETRACT);
        m_lastCommandTimeMs = timeMs;
    }

    // Check for a timeout
    if (millis() - this->time_entered_state > GeneralConfig::MaxZeroTimeMs) {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Failed to zero the grid fins in time, returning to default.");
        m_status.newFlag(TOASTER_FLAGS::ERROR_ZERO_TIMEOUT);
        return std::make_unique<Default>(m_system, m_status);
    }

    return nullptr;
};

void Zero::exit(){
    m_system.toaster.stepperCommand(Toaster::StepperCommandPayload::HOLD);
    Types::TOASTER_TYPES::State_t::exit();
};