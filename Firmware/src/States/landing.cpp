#include "States/landing.h"

Landing::Landing(System &system) : 
        State(SYSTEM_FLAG::STATE_LANDING, system.systemstatus),
        _system(system) {};

void Landing::initialize()
{
    State::initialize();
    _system.commandhandler.enableCommands({
                                            Commands::ID::Telemetry,
                                          });
};

Types::CoreTypes::State_ptr_t Landing::update()
{
    return nullptr;
};

void Landing::exit()
{
    
    State::exit();
    _system.commandhandler.resetCommands();

};