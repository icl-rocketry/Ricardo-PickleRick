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

    const Eigen::Vector3f start_pos(0.0f, 0.0f, 0.0f);
    const Eigen::Vector3f end_pos(1.0f, 0.0f, 0.0f);
    const float duration_s = 5.0f;

    m_trajectory_active = m_position_trajectory.configure(start_pos, end_pos, duration_s);
    m_trajectory_start_ms = millis();
    _system.controller.setPositionControlEnabled(m_trajectory_active);
};

Types::CoreTypes::State_ptr_t Flight::update()
{
    auto current_Data = _system.estimator.getData();

    const uint32_t current_time_ms = millis();

    // if ((current_time_ms - _system.controller.getStartTime()) > 2500
    //     || current_Data.rocketEulerAngles(1) > 30.0f
    //     || current_Data.rocketEulerAngles(2) > 30.0f) {
    //     return std::make_unique<Landing>(_system);
    // }

    auto quaternion = current_Data.rocketOrientation.cast<double>();
    auto angular_rates = current_Data.angularRates;
    auto position = current_Data.position;
    auto velocity = current_Data.velocity;
    
    _system.controller.setBatteryVoltage(_system.powermonitor.getBatteryVoltage(),_system.powermonitor.fresh());

    if (m_trajectory_active) {
        const float elapsed_s = (current_time_ms - m_trajectory_start_ms) * 1e-3f;
        const Trajectory::TrajectoryPoint target = m_position_trajectory.sample(elapsed_s);

        _system.controller.setPositionTarget(target.position,
                                             target.velocity,
                                             target.acceleration);
        m_trajectory_active = !target.finished;
    }

    _system.controller.update(quaternion, angular_rates, position, velocity, true);

    return nullptr;
};

void Flight::exit()
{
    
    State::exit();
    _system.controller.stop();
    _system.commandhandler.resetCommands();

};
