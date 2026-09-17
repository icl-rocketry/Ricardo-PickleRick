#include "armed.h"

#include <libriccore/fsm/state.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include "Config/services_config.h"
#include "Config/commands_config.h"

#include "system.h"

Armed::Armed(System& system):
State(SYSTEM_FLAG::STATE_ARMED, system.systemstatus),
_system(system)
{};

void Armed::initialize(){
    State::initialize();
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered armed state.");
};

Types::CoreTypes::State_ptr_t Armed::update(){

    return nullptr;
};

void Armed::exit(){
    Types::CoreTypes::State_t::exit();
};