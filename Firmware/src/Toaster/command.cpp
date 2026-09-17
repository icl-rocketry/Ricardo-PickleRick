#include "command.h"

#include <libriccore/fsm/state.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include "Config/services_config.h"
#include "Config/commands_config.h"

#include "system.h"

Command::Command(System& system):
State(SYSTEM_FLAG::STATE_COMMAND, system.systemstatus),
_system(system)
{};

void Command::initialize(){
    State::initialize();
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered command state.");
};

Types::CoreTypes::State_ptr_t Command::update(){

    return nullptr;
};

void Command::exit(){
    Types::CoreTypes::State_t::exit();
};