
#include "States/preflight.h"

Preflight::Preflight(System& system):
        State(SYSTEM_FLAG::STATE_PREFLIGHT,system.systemstatus),
        _system(system) {};

void Preflight::initialize(){
    State::initialize();
    _system.commandhandler.enableCommands({
                                           Commands::ID::Set_Home,
                                           Commands::ID::Stop_Logging,
                                           Commands::ID::Enter_Flight
                                          });    

};

Types::CoreTypes::State_ptr_t Preflight::update()
{
    auto current_Data = _system.estimator.getData(); 
    
    auto quat = current_Data.rocketOrientation; 

    // if (millis() > 15000) {
    //     return std::make_unique<Flight>(_system);
    // }

    Eigen::Matrix<float, 1, 7> inputMatrix = {
        quat.w(),
        quat.x(),
        quat.y(),
        quat.z(),
        current_Data.angularRates(0),
        current_Data.angularRates(1),
        current_Data.angularRates(2),
    };    
    

    _system.controller.update(inputMatrix, false);

    return nullptr;
};

void Preflight::exit(){
    State::exit();
    _system.commandhandler.resetCommands();
};