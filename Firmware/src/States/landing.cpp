#include "States/landing.h"

#include <algorithm>

#include "Config/landing_config.h"

Landing::Landing(System &system) : 
        State(SYSTEM_FLAG::STATE_LANDING, system.systemstatus),
        _system(system) {};

void Landing::initialize()
{
    State::initialize();
    _system.commandhandler.enableCommands({
                                            Commands::ID::Telemetry,
                                          });

    const auto current_data = _system.estimator.getData();
    const Eigen::Vector3f start_position_ned = current_data.position;

    m_target_position_ned = start_position_ned;
    m_target_position_ned(2) = -LandingConfig::CutoffHeightAboveGroundM;
    m_phase = Phase::Descent;
    m_phase_start_ms = millis();
    m_controller_stopped = false;

    _system.controller.setPositionControlEnabled(true);
    _system.controller.setPositionTarget(start_position_ned,
                                         Eigen::Vector3f::Zero(),
                                         Eigen::Vector3f::Zero());

    if (reachedCutoffHeight(start_position_ned)) {
        beginThrottleFade();
        return;
    }

    const float descent_distance_m = (m_target_position_ned - start_position_ned).norm();
    const float descent_duration_s = std::max(LandingConfig::MinDescentDurationS,
                                              descent_distance_m / LandingConfig::DescentRateMps);

    m_descent_active = m_descent_trajectory.configure(start_position_ned,
                                                      m_target_position_ned,
                                                      descent_duration_s,
                                                      LandingConfig::DescentAccelFraction);

    if (!m_descent_active) {
        beginThrottleFade();
    }
};

Types::CoreTypes::State_ptr_t Landing::update()
{
    const auto current_data = _system.estimator.getData();
    const uint32_t current_time_ms = millis();

    _system.controller.setBatteryVoltage(_system.powermonitor.getBatteryVoltage(),
                                         _system.powermonitor.fresh());

    if (m_phase == Phase::Descent) {
        if (reachedCutoffHeight(current_data.position)) {
            beginThrottleFade();
        } else {
            const float elapsed_s = (current_time_ms - m_phase_start_ms) * 1e-3f;
            const Trajectory::TrajectoryPoint target = m_descent_trajectory.sample(elapsed_s);

            _system.controller.setPositionTarget(target.position,
                                                 target.velocity,
                                                 target.acceleration);
            _system.controller.update(current_data.orientation.cast<double>(),
                                      current_data.angularRates,
                                      current_data.position,
                                      current_data.velocity,
                                      true);
        }
    }

    if (m_phase == Phase::ThrottleFade) {
        updateThrottleFade(millis());
    }

    return nullptr;
};

void Landing::exit()
{
    
    State::exit();
    stopControllerOnce();
    _system.commandhandler.resetCommands();

};

void Landing::beginThrottleFade()
{
    m_fade_start_output = _system.controller.getOutputValues();
    m_phase_start_ms = millis();
    m_phase = Phase::ThrottleFade;
    _system.controller.setPositionControlEnabled(false);
}

void Landing::updateThrottleFade(uint32_t current_time_ms)
{
    const uint32_t elapsed_ms = current_time_ms - m_phase_start_ms;
    const float fade_fraction = std::clamp(
        static_cast<float>(elapsed_ms) / static_cast<float>(LandingConfig::ThrottleFadeDurationMs),
        0.0f,
        1.0f);
    const float thrust_scale = 1.0f - fade_fraction;

    Eigen::Vector4f output = m_fade_start_output;
    output(2) = m_fade_start_output(2) * thrust_scale;
    output(3) = m_fade_start_output(3) * thrust_scale;
    _system.controller.setManualOutput(output, true);

    if (elapsed_ms >= LandingConfig::ThrottleFadeDurationMs) {
        _system.controller.setManualOutput(Eigen::Vector4f::Zero(), true);
        stopControllerOnce();
        m_phase = Phase::Complete;
    }
}

void Landing::stopControllerOnce()
{
    if (!m_controller_stopped) {
        _system.controller.stop();
        m_controller_stopped = true;
    }
}

bool Landing::reachedCutoffHeight(const Eigen::Vector3f& position_ned) const
{
    return position_ned(2) >=
           m_target_position_ned(2) - LandingConfig::CutoffHeightToleranceM;
}
