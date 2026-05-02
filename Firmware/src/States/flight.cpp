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

    float pitch = current_Data.rocketEulerAngles(1);
    float yaw = current_Data.rocketEulerAngles(2);

    // if ((current_time - t ) > 2500 || pitch > 30.0f || yaw > 30.0f) {
    //     return std::make_unique<Landing>(_system);
    // }

    auto quaternion = current_Data.rocketOrientation.cast<double>();
    auto angular_rates = current_Data.angularRates;
    auto position = current_Data.position;
    auto velocity = current_Data.velocity;
    
    _system.controller.setBatteryVoltage(_system.powermonitor.getBatteryVoltage(),_system.powermonitor.fresh());

    _system.controller.update(quaternion, angular_rates, position, velocity, true);

    return nullptr;
};

void Flight::exit()
{
    
    State::exit();
    _system.controller.stop();
    _system.commandhandler.resetCommands();

};