
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
    auto current_Data_sensors = _system.sensors.getData();
    
    auto quat = current_Data.orientation; 

    Eigen::Matrix<float, 1, 7> inputMatrix = {
        quat.w(),
        quat.x(),
        quat.y(),
        quat.z(),
        current_Data_sensors.accelgyro.gx,
        current_Data_sensors.accelgyro.gy,
        current_Data_sensors.accelgyro.gz,
    };
    

    _system.controller.update(inputMatrix, false);

    if (millis() > 15000) {
        return std::make_unique<Flight>(_system);
    }
    return nullptr;
};

void Preflight::exit(){
    State::exit();
    _system.commandhandler.resetCommands();
};