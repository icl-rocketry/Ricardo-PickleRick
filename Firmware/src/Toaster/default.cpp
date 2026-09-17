#include "default.h"

#include <libriccore/fsm/state.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include "Config/services_config.h"
#include "Config/commands_config.h"

#include "system.h"

Default::Default(System& system):
State(SYSTEM_FLAG::STATE_COMMAND, system.systemstatus),
_system(system)
{};

void Default::initialize(){
    State::initialize();

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered default state.");
};

Types::CoreTypes::State_ptr_t Default::update(){

    return nullptr;
};

void Default::exit(){
    Types::CoreTypes::State_t::exit();
};