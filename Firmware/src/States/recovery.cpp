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
    _system.tunezhandler.play(MelodyLibrary::tetris,true); // play startup sound
    _system.enginehandler.shutdownAllEngines();
};


Types::CoreTypes::State_ptr_t Recovery::update(){

    _system.enginehandler.update();
    _system.guidedhandler.update();
    _system.eventhandler.update(_system.estimator.getData());
    _system.controllerhandler.update(_system.estimator.getData());
    
    if (_system.estimator.getData().position(2) < GuidedConfig::guided_alt_start && !_system.systemstatus.flagSet(SYSTEM_FLAG::FLIGHTPHASE_GUIDED))
    {
        _system.systemstatus.newFlag(SYSTEM_FLAG::FLIGHTPHASE_GUIDED,"Entered Guided Phase");
        _system.systemstatus.deleteFlag(SYSTEM_FLAG::FLIGHTPHASE_APOGEE);
        _system.guidedhandler.startAllGuided();
    }

    return nullptr;
};

void Recovery::exit(){
    State::exit();
    _system.tunezhandler.clear(); // stop looping zelda
    _system.commandhandler.resetCommands();
};