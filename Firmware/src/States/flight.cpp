#include "States/flight.h"

#include <cmath>

#include "Config/general_config.h"
#include "Config/loggerhandler_config.h"

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

    if (GeneralConfig::ThrottleRampTestEnabled) { //code for the throttle load cell testing
        m_trajectory_active = false;
        _system.controller.setPositionControlEnabled(false);
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Throttle profile test enabled: running full voltage/thrust calibration profile");
        return;
    }

    const Eigen::Vector3f start_pos(0.0f, 0.0f, 0.0f); //NED frame
    const Eigen::Vector3f end_pos(0.0f, 0.0f, -0.8f); //NED frame
    const float duration_s = 2.0f;
    m_trajectory_active = m_position_trajectory.configure(start_pos, end_pos, duration_s);
    m_trajectory_start_ms = millis();

    _system.controller.setPositionTarget(start_pos, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
    _system.controller.setPositionControlEnabled(true);//set to true to enable position control
};

Types::CoreTypes::State_ptr_t Flight::update()
{
    auto current_Data = _system.estimator.getData();

    const uint32_t current_time_ms = millis();

    _system.controller.setBatteryVoltage(_system.powermonitor.getBatteryVoltage(),_system.powermonitor.fresh());

    if (GeneralConfig::ThrottleRampTestEnabled) {
        _system.controller.updateThrottleProfileTest(true);
        return nullptr;
    }
   // If the drone is tilted more than 25 degrees from upright, transition to landing state to prevent flyaway
    constexpr float kMaxTiltRad = 25.0f * DEG_TO_RAD;
    const Eigen::Vector3f thrust_axis_world =
        current_Data.orientation.normalized() * Eigen::Vector3f::UnitX();
    const Eigen::Vector3f upright_thrust_world(0.0f, 0.0f, -1.0f);

    if (thrust_axis_world.dot(upright_thrust_world) < std::cos(kMaxTiltRad)) {
        return std::make_unique<Landing>(_system);
    }

    auto quaternion = current_Data.orientation.cast<double>();
    auto angular_rates = current_Data.angularRates;
    auto position = current_Data.position; //NED
    auto velocity = current_Data.velocity; //NED
    
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
