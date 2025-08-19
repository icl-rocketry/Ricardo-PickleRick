#include "flight.h"

Flight::Flight(System &system) : 
        State(SYSTEM_FLAG::STATE_FLIGHT, system.systemstatus),
        _system(system) {};

void Flight::initialize()
{
    State::initialize();
    _system.commandhandler.enableCommands({
                                            Commands::ID::Telemetry,
                                            Commands::ID::Enter_Hard_Abort,
                                            Commands::ID::Enter_Soft_Abort,
                                            Commands::ID::Enter_Land,
                                          });
    _system.estimator.setFlightTime(millis());
    _system.controller.start();
};

Types::CoreTypes::State_ptr_t Flight::update()
{
    auto current_Data = _system.estimator.getData(); 
    float roll = current_Data.eulerAngles[0];
    float pitch = current_Data.eulerAngles[1];

    // float x = current_Data.position[0];
    // float y = current_Data.position[1];
    // float z = current_Data.position[2];

    uint32_t t = current_Data.flightTime;
    uint32_t current_time = millis();       

    // Condition A

    // Also Implement a chack for low battery !!!!!!
    if ((current_time - t ) > 1500) {
        return std::make_unique<Preflight>(_system);
    }

    // Condition D
    if ((abs(roll) > 3.142/18) || (abs(pitch) > 3.142/18)) // || (abs(x) > 5) || (abs(y) > 5) || (abs(z) > 10))
    { 
        return std::make_unique<Hard_Abort>(_system);
    }


    // Condition E
    // if ((abs(x) > 3) || (abs(y) > 3) || (abs(z) > 6))
    // { 
    //     return std::make_unique<Soft_Abort>(_system);
    // }


    Eigen::Matrix<float,1,13> inputMatrix = {
        current_Data.position(0),
        current_Data.position(1),
        current_Data.position(2),
        current_Data.velocity(0),
        current_Data.velocity(1),
        current_Data.velocity(2),
        static_cast<float>(current_Data.eulerAngles[0]),
        static_cast<float>(current_Data.eulerAngles[1]),
        static_cast<float>(current_Data.eulerAngles[2]),
        current_Data.angularRates(0),
        current_Data.angularRates(1),
        current_Data.angularRates(2),
        (current_time - t)/1000.0f // flight time in seconds
    };
    
    // Eigen::Matrix<float,1,12> inputMatrix = {
    //     0.1,
    //     0.1,
    //     0.1,
    //     0.1,
    //     0.1,
    //     0.1,        
    //     0.1,
    //     0.1,
    //     0.1,        
    //     0.1,
    //     0.1,
    //     0.1
    // };
    _system.controller.update(inputMatrix);

    return nullptr;
};

void Flight::exit()
{
    State::exit();
    _system.controller.stop();
    _system.commandhandler.resetCommands();
};