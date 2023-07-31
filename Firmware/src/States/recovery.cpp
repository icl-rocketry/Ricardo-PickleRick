#include "recovery.h"

#include "system.h"
#include "Config/types.h"
#include "Config/systemflags_config.h"
#include "Config/commands_config.h"

#include "Sound/Melodies/melodyLibrary.h"


Recovery::Recovery(System& system):
State(SYSTEM_FLAG::STATE_RECOVERY,system.systemstatus),
_system(system)
{};

void Recovery::initialize(){
    State::initialize();
    _system.commandhandler.enableCommands({Commands::ID::Reset});
    _system.tunezhandler.play(MelodyLibrary::zeldatheme,true); // play startup sound
    _system.enginehandler.shutdownAllEngines();
};


Types::CoreTypes::State_ptr_t Recovery::update(){

    _system.enginehandler.update();
    _system.controllerhandler.update(_system.estimator.getData());
    _system.eventhandler.update(_system.estimator.getData());
    
    return nullptr;
};

void Recovery::exit(){
    State::exit();
    _system.tunezhandler.clear(); // stop looping zelda
    _system.commandhandler.resetCommands();
};