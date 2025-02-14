#include "landing.h"

#include <memory>

#include <libriccore/fsm/state.h>
#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include "Config/commands_config.h"

#include "system.h"


Landing::Landing(System &system) : 
    State(SYSTEM_FLAG::STATE_LANDING, system.systemstatus),
    _system(system) {};

void Landing::initialize()
{
    State::initialize();

    _system.commandhandler.enableCommands({
                                           Commands::ID::Telemetry,
                                           Commands::ID::Enter_Hard_Abort,
                                           Commands::ID::Enter_Soft_Abort,
                                          });
};

Types::CoreTypes::State_ptr_t Landing::update()
{
    auto current_Data = _system.estimator.getData(); 
    auto roll = current_Data.eulerAngles[0];
    auto pitch = current_Data.eulerAngles[1];
    auto yaw = current_Data.eulerAngles[2];

    auto x = current_Data.position[0];
    auto y = current_Data.position[1];
    auto z = current_Data.position[2];

    // Condition F
    if ((roll > 3.142/2) || (pitch > 3.142/2) || (abs(x) > 5) || (abs(y) > 5) || (abs(z) > 10))
    { 
        return std::make_unique<Hard_Abort>(_system);
    }
    else
    {
        return nullptr;
    }

    // Condition G
    if ((abs(x) > 3) || (abs(y) > 3) || (abs(z) > 6))
    { 
        return std::make_unique<Soft_Abort>(_system);
    }
    else
    {
        return nullptr;
    }
};

void Landing::exit()
{
    State::exit();
    _system.commandhandler.resetCommands();
};

