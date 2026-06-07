
#include "States/preflight.h"

Preflight::Preflight(System& system):
        State(SYSTEM_FLAG::STATE_PREFLIGHT,system.systemstatus),
        _system(system) {};

void Preflight::initialize(){
    State::initialize();
    _system.commandhandler.enableCommands({
                                           Commands::ID::Set_Home,
                                           Commands::ID::Calibrate,
                                           Commands::ID::Enter_Flight
                                          });    

};

Types::CoreTypes::State_ptr_t Preflight::update()
{
    auto current_Data = _system.estimator.getData(); 

    // if (millis() > 5000) { //COMMENT OUT TO DISABLE AUTOSTART
    //     return std::make_unique<Flight>(_system);
    // }

    auto quaternion = current_Data.orientation.cast<double>();
    auto angular_rates = current_Data.angularRates;
    auto position = current_Data.position;
    auto velocity = current_Data.velocity;
    _system.controller.setPositionControlEnabled(false);
    _system.controller.update(quaternion, angular_rates, position, velocity, false);

    return nullptr;
};

void Preflight::exit(){
    State::exit();
    _system.commandhandler.resetCommands();
};
