#include "launch.h"

#include <memory>

#include "flight.h"

#include "system.h"

#include "Sound/Melodies/melodyLibrary.h"

#include "Config/types.h"
#include "Config/systemflags_config.h"
#include "Config/commands_config.h"

Launch::Launch(System &system) : State(SYSTEM_FLAG::STATE_LAUNCH, system.systemstatus),
                                 _system(system){};

void Launch::initialize()
{
    State::initialize();

    _system.commandhandler.enableCommands({Commands::ID::Ignition,
                                           Commands::ID::Launch_Abort,
                                           Commands::ID::Liftoff_Override});

    
    _system.tunezhandler.play(MelodyLibrary::confirmation);
    _system.tunezhandler.play(MelodyLibrary::miichannel,true);

    // arm deployers and engines

    _system.deploymenthandler.armComponents();
    _system.enginehandler.armComponents();
};

Types::CoreTypes::State_ptr_t Launch::update()
{

    _system.eventhandler.update(_system.estimator.getData());

    // perform flight check
    int deployers_in_error = _system.deploymenthandler.flightCheck();
    int engines_in_error = _system.enginehandler.flightCheck();
    _system.enginehandler.update();

    if ((deployers_in_error == 0) && (engines_in_error == 0))
    {
        if (_system.systemstatus.flagSet(SYSTEM_FLAG::ERROR_FLIGHTCHECK))
        {
            _system.systemstatus.deleteFlag(SYSTEM_FLAG::ERROR_FLIGHTCHECK);

        }
    }
    else
    {
        if (!_system.systemstatus.flagSet(SYSTEM_FLAG::ERROR_FLIGHTCHECK))
        {
            _system.systemstatus.newFlag(SYSTEM_FLAG::ERROR_FLIGHTCHECK);
            
        }
    }

    // if (!_system.systemstatus.flagSet(SYSTEM_FLAG::ERROR_FLIGHTCHECK) && _system.estimator.getData().acceleration(2) < -1){ // launch acceleration threshold comparison of down acceleration with a threshold of 1.5 g idk if this is okay lol?
    if (_system.estimator.getData().acceleration(2) < -(2*9.81) && _system.estimator.getData().position(2) < -50)
    { // launch acceleration threshold comparison of down acceleration with a threshold of 1.5 g idk if this is okay lol?
        _system.estimator.setLiftoffTime(millis());
        return std::make_unique<Flight>(_system);
    }
    else
    {

        return nullptr; // loopy loop
    }
};

void Launch::exit()
{
    State::exit();
    _system.commandhandler.resetCommands();
    _system.tunezhandler.clear();

};
