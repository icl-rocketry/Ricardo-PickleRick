#include "hardAbort.h"

#include <memory>

#include <libriccore/fsm/state.h>
#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include "Config/commands_config.h"

#include "system.h"


Hard_Abort::Hard_Abort(System &system) : 
            State(SYSTEM_FLAG::STATE_HARD_ABORT, system.systemstatus),
             _system(system) {};

void Hard_Abort::initialize()
{
    State::initialize();

    _system.commandhandler.enableCommands({Commands::ID::Telemetry,
                                           Commands::ID::Reset
                                          });

    /*
    Kill all engines
    
    */
    
};

Types::CoreTypes::State_ptr_t Hard_Abort::update()
{
    return nullptr;
};

void Hard_Abort::exit()
{
    State::exit();
    _system.commandhandler.resetCommands();
};

