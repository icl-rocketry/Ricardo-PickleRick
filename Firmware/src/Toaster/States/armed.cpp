#include "armed.h"

#include <libriccore/fsm/state.h>

#include "Config/commands_config.h"
#include "Toaster/types.h"
#include "deploy.h"
#include "system.h"

Armed::Armed(System& system, Types::TOASTER_TYPES::SystemStatus_t& status):
    State(TOASTER_FLAGS::STATE_ARMED, status),
    m_system(system),
    m_status(status) {};

void Armed::initialize(){
    State::initialize();

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered Armed state.");

    m_system.toaster.stepperEnable();
};

Types::TOASTER_TYPES::State_ptr_t Armed::update(){
    // Check launch conditions
    if (m_system.estimator.getData().acceleration(2) < -(2*9.81) && m_system.estimator.getData().position(2) < -50) {
        m_system.estimator.setLiftoffTime(millis());
        return std::make_unique<Deploy>(m_system, m_status);
    }

    return nullptr;
};

void Armed::exit(){
    Types::TOASTER_TYPES::State_t::exit();
};