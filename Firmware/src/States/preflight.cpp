
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
    // if (millis() > 5000) { //COMMENT OUT TO DISABLE AUTOSTART
    //     return std::make_unique<Flight>(_system);
    // }

    _system.controller.setPositionControlEnabled(false);
    // Keep downstream motor controllers quiet if the flight computer resets into preflight.
    _system.controller.setManualOutput(Eigen::Vector4f::Zero(), true);

    return nullptr;
};

void Preflight::exit(){
    State::exit();
    _system.commandhandler.resetCommands();
};
