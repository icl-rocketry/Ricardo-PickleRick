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
#include "preflight.h"

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

    _system.estimator.setFlightTime(millis());
};

Types::CoreTypes::State_ptr_t Landing::update()
{
    auto current_Data = _system.estimator.getData(); 
    auto roll = current_Data.eulerAngles[0];
    auto pitch = current_Data.eulerAngles[1];

    // auto x = current_Data.position[0];
    // auto y = current_Data.position[1];
    // auto z = current_Data.position[2];

    uint32_t t = current_Data.flightTime; // tim @ start landing
    uint32_t current_time = millis(); 
    uint32_t time_landing = current_time - t;
    float scaling_factor = time_landing/current_time; 


    // Condition F
    if ((abs(roll) > 3.142/2) || (abs(pitch) > 3.142/2)) // || (abs(x) > 5) || (abs(y) > 5) || (abs(z) > 10))
    { 
        return std::make_unique<Hard_Abort>(_system);
    }

    // // Condition G
    // if ((abs(x) > 3) || (abs(y) > 3) || (abs(z) > 6))
    // { 
    //     return std::make_unique<Soft_Abort>(_system);
    // }

    // Check if landed (uses vertical acceleration, maybe fix?)
    // if ((_system.pid1.getThrust1() == 0) && ((_system.pid1.getThrust2() == 0)) && (current_Data.acceleration[2] < -0.9)) {
    //     return std::make_unique<Preflight>(_system);
    // }

    if ((current_time - t ) > 10000) {
        return std::make_unique<Preflight>(_system);
        
    }

    // Eigen::Matrix<float,1,12> inputMatrix = {
    //     current_Data.position(0),
    //     current_Data.position(1),
    //     current_Data.position(2),
    //     current_Data.velocity(0),
    //     current_Data.velocity(1),
    //     current_Data.velocity(2),
    //     static_cast<float>(current_Data.eulerAngles[0] * (180 / PI)),
    //     static_cast<float>(current_Data.eulerAngles[1] * (180 / PI)),
    //     static_cast<float>(current_Data.eulerAngles[2] * (180 / PI)),
    //     current_Data.angularRates(0),
    //     current_Data.angularRates(1),
    //     current_Data.angularRates(2)
    // };
    // _system.controller.update(inputMatrix);


    return nullptr;
};

void Landing::exit()
{
    State::exit();
    _system.commandhandler.resetCommands();
    
};

