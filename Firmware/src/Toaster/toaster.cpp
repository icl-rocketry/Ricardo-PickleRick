#include "toaster.h"

#include "States/default.h"
#include "States/armed.h"
#include "States/deploy.h"
#include "States/command.h"
#include "States/zero.h"

void Toaster::setup() {
    // Start in default status
    m_toasterMachine.initalize(std::make_unique<Default>(m_system, m_toasterStatus));

    // Initialise the stepper enable pin
    // Enable pin is active low
    pinMode(m_enablePin, OUTPUT);
    digitalWrite(m_enablePin, HIGH);

    m_networkmanager.registerService(static_cast<uint8_t>(Services::ID::TOASTER), this->getThisNetworkCallback());
}

void Toaster::update() {
    m_toasterMachine.update();
}

void Toaster::apogeeDetected() {
    // Ensure module is armed
    if (m_toasterStatus.getStatus() != static_cast<uint32_t>(TOASTER_FLAGS::STATE_ARMED)) {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Toaster not armed, cant deploy.");
        return;
    }

    // Transition into deployed state.
    m_toasterMachine.changeState(std::make_unique<Deploy>(m_system, m_toasterStatus));
}

void Toaster::arm_base(int32_t arg) {
    if (m_toasterStatus.getStatus() != static_cast<uint32_t>(TOASTER_FLAGS::STATE_DEFAULT)) {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Toaster not in default, cant arm.");
        return;
    }

    switch (arg) {
    case 0: {
        // Transition to armed state
        m_toasterMachine.changeState(std::make_unique<Armed>(m_system, m_toasterStatus));
        break;
    }
    case 1: {
        // Zero the stepper motor against the endpoint before arming
        m_toasterMachine.changeState(std::make_unique<Zero>(m_system, m_toasterStatus));
        break;
    }
    case 2: {
        m_toasterMachine.changeState(std::make_unique<Command>(m_system, m_toasterStatus));
        break;
    }
    }
}

void Toaster::disarm_base()
{
}

void Toaster::execute_base(int32_t arg)
{
}
