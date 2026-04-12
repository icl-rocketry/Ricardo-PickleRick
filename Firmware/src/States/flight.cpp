#include "States/flight.h"

Flight::Flight(System &system) : 
        State(SYSTEM_FLAG::STATE_FLIGHT, system.systemstatus),
        _system(system) {};

void Flight::initialize()
{
    State::initialize();
    _system.commandhandler.enableCommands({
                                            Commands::ID::Telemetry,
                                            Commands::ID::Enter_Landing,
                                          });
    _system.controller.start();
};

Types::CoreTypes::State_ptr_t Flight::update()
{
    auto current_Data = _system.estimator.getData(); 
    
    uint32_t t = _system.controller.getStartTime();
    uint32_t current_time = millis();       

    if ((current_time - t ) > 3000) {
        return std::make_unique<Landing>(_system);
    }

    auto quat = current_Data.rocketOrientation; 

    Eigen::Matrix<float, 1, 7> inputMatrix = {
        quat.w(),
        quat.x(),
        quat.y(),
        quat.z(),
        current_Data.angularRates(0),
        current_Data.angularRates(1),
        current_Data.angularRates(2),
    };    

    _system.controller.update(inputMatrix, true);

    return nullptr;
};

void Flight::exit()
{
    
    State::exit();
    _system.controller.stop();
    _system.commandhandler.resetCommands();

};