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

Zero::Zero(System& system, Types::TOASTER_TYPES::SystemStatus_t& status, bool toDefault):
    State(TOASTER_FLAGS::STATE_ZERO, status),
    m_system(system),
    m_status(status),
    m_toDefault(toDefault) {};

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
        // Transition to next state
        m_system.toaster.stepperCommand(Toaster::StepperCommandPayload::HOLD);
        m_status.deleteFlag(TOASTER_FLAGS::ERROR_ZERO_TIMEOUT);

        if (m_toDefault) {
            return std::make_unique<ToasterDefault>(m_system, m_status);
        } else {
            return std::make_unique<Armed>(m_system, m_status);
        }
    }

    const uint64_t timeMs = millis();

    if (timeMs - m_lastCommandTimeMs >= GeneralConfig::CommandDeltaMs) {
        m_system.toaster.stepperCommand(Toaster::StepperCommandPayload::RETRACT);
        m_lastCommandTimeMs = timeMs;
    }

    // Check for a timeout
    if (millis() - this->time_entered_state > GeneralConfig::MaxZeroTimeMs) {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Failed to zero the grid fins in time, returning to default.");
        m_status.newFlag(TOASTER_FLAGS::ERROR_ZERO_TIMEOUT);
        m_system.toaster.stepperCommand(Toaster::StepperCommandPayload::HOLD);
        return std::make_unique<ToasterDefault>(m_system, m_status);
    }

    return nullptr;
};

void Zero::exit(){
    Types::TOASTER_TYPES::State_t::exit();
};