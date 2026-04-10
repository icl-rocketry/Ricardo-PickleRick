#include "States/flight.h"

Flight::Flight(System &system) : 
        State(SYSTEM_FLAG::STATE_FLIGHT, system.systemstatus),
        _system(system) {};

void Flight::initialize()
{
    State::initialize();
    _system.commandhandler.enableCommands({
                                            Commands::ID::Telemetry,
                                            Commands::ID::Enter_Preflight,
                                          });
    _system.controller.start();
};

Types::CoreTypes::State_ptr_t Flight::update()
{
    auto current_Data = _system.estimator.getData(); 
    auto current_Data_sensors = _system.sensors.getData();
    
    uint32_t t = _system.controller.getStartTime();
    uint32_t current_time = millis();       

    if ((current_time - t ) > 1000) {
        return std::make_unique<Preflight>(_system);
    }

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

    _system.controller.update(inputMatrix, true);

    return nullptr;
};

void Flight::exit()
{
    
    State::exit();
    _system.controller.stop();
    _system.commandhandler.resetCommands();

};