#include "default.h"

#include <libriccore/fsm/state.h>

#include "Config/commands_config.h"

#include "Toaster/types.h"

#include "system.h"

ToasterDefault::ToasterDefault(System& system, Types::TOASTER_TYPES::SystemStatus_t& status):
    State(TOASTER_FLAGS::STATE_DEFAULT, status),
    m_system(system) {};

void ToasterDefault::initialize(){
    State::initialize();

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered Default state.");

    m_system.toaster.stepperDisable();
};

Types::TOASTER_TYPES::State_ptr_t ToasterDefault::update(){

    return nullptr;
};

void ToasterDefault::exit(){
    Types::TOASTER_TYPES::State_t::exit();
};