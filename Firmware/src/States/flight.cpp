#include "States/flight.h"

#include <cmath>

#include "Config/general_config.h"
#include "Config/loggerhandler_config.h"

namespace {
Eigen::Vector3f toVector(const FlightTrajectoryConfig::WaypointNed& waypoint)
{
    return Eigen::Vector3f(waypoint.north_m, waypoint.east_m, waypoint.down_m);
}
}

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
    m_tilt_power_cut = false;
    _system.controller.start();

    if (GeneralConfig::ThrottleRampTestEnabled) { //code for the throttle load cell testing
        m_trajectory_active = false;
        _system.controller.setPositionControlEnabled(false);
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Throttle profile test enabled: running full voltage/thrust calibration profile");
        return;
    }

    m_trajectory_active = configureTrajectoryLeg(0);

    if (m_trajectory_active) {
        const auto& first_leg = FlightTrajectoryConfig::Legs[m_trajectory_leg_index];
        _system.controller.setPositionTarget(toVector(first_leg.start_ned_m),
                                             Eigen::Vector3f::Zero(),
                                             Eigen::Vector3f::Zero());
    }
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
    if (m_tilt_power_cut) {
        _system.controller.cutEnginePower();
        return nullptr;
    }

    // If the drone is tilted more than 25 degrees from upright, cut engine power to prevent flyaway.
    constexpr float kMaxTiltRad = 25.0f * DEG_TO_RAD;
    const Eigen::Vector3f thrust_axis_world =
        current_Data.orientation.normalized() * Eigen::Vector3f::UnitX();
    const Eigen::Vector3f upright_thrust_world(0.0f, 0.0f, -1.0f);

    if (thrust_axis_world.dot(upright_thrust_world) < std::cos(kMaxTiltRad)) {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(
            "Tilt safeguard triggered: cutting engine power");
        _system.controller.setPositionControlEnabled(false);
        _system.controller.cutEnginePower();
        m_tilt_power_cut = true;
        return nullptr;
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

        if (target.finished) {
            if (!configureTrajectoryLeg(m_trajectory_leg_index + 1)) {
                RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(
                    "Flight trajectory complete: entering landing");
                return std::make_unique<Landing>(_system);
            }
        }
    }
    _system.controller.update(quaternion, angular_rates, position, velocity, true);

    return nullptr;
};

void Flight::exit()
{
    
    State::exit();
    if (m_tilt_power_cut) {
        _system.controller.stop();
    }
    _system.commandhandler.resetCommands();

};

bool Flight::configureTrajectoryLeg(std::size_t leg_index)
{
    if (leg_index >= FlightTrajectoryConfig::LegCount) {
        return false;
    }

    const auto& leg = FlightTrajectoryConfig::Legs[leg_index];
    m_trajectory_leg_index = leg_index;
    m_trajectory_start_ms = millis();

    return m_position_trajectory.configure(toVector(leg.start_ned_m),
                                           toVector(leg.finish_ned_m),
                                           leg.duration_s,
                                           leg.accel_fraction);
}
