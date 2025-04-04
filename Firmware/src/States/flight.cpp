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
    float yaw = current_Data.eulerAngles[2];

    float x = current_Data.position[0];
    float y = current_Data.position[1];
    float z = current_Data.position[2];

    uint32_t t = current_Data.flightTime;
    uint32_t current_time = millis();       

    // Condition A

    // Also Implement a chack for low battery !!!!!!
    if ((current_time - t ) > 150000) {
        return std::make_unique<Landing>(_system);
    }

    // Condition D
    if ((abs(roll) > 3.142/2) || (abs(pitch) > 3.142/2)) // || (abs(x) > 5) || (abs(y) > 5) || (abs(z) > 10))
    { 
        return std::make_unique<Hard_Abort>(_system);
    }


    // // Condition E
    // if ((abs(x) > 3) || (abs(y) > 3) || (abs(z) > 6))
    // { 
    //     return std::make_unique<Soft_Abort>(_system);
    // }


    Eigen::Matrix<float,1,6> inputMatrix = {
        current_Data.position(0),
        current_Data.position(1),
        current_Data.position(2),
        static_cast<float>(current_Data.eulerAngles[0] * (180 / PI)),
        static_cast<float>(current_Data.eulerAngles[1] * (180 / PI)),
        static_cast<float>(current_Data.eulerAngles[2] * (180 / PI))
    };
    
    _system.controller.update(inputMatrix);

    return nullptr;
};

void Flight::exit()
{
    State::exit();
    _system.controller.stop();
    _system.commandhandler.resetCommands();
};