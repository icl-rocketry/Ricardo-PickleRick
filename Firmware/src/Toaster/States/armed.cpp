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

    // TODO: Only arm at apogee?
    m_system.toaster.stepperCommand(Toaster::StepperCommandPayload::HOLD);
    m_system.toaster.stepperEnable();
};

Types::TOASTER_TYPES::State_ptr_t Armed::update(){
    const auto& measurement = m_system.estimator.getData();

    // Check launch conditions
    if (!m_launchDetected && (measurement.acceleration(2) < -(2*9.81) && measurement.position(2) < -50)) {
        m_system.estimator.setLiftoffTime(millis());
        m_launchDetected = true;
    }

    // If launched, detect apogee conditions
    if (m_launchDetected && m_system.apogeedetect.checkApogee(-measurement.position(2), -measurement.velocity(2), millis()).reached) {
        m_system.estimator.setApogeeTime(millis());
        return std::make_unique<Deploy>(m_system, m_status);
    }

    return nullptr;
};

void Armed::exit(){
    Types::TOASTER_TYPES::State_t::exit();
};