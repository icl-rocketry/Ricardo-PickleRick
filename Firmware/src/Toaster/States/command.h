
#pragma once

#include <memory>

#include "Toaster/Target/targetRollGenerator.h"
#include "Toaster/PID/pid.h"
#include "Toaster/Util/angleWrapper.h"
#include "Toaster/types.h"
#include "system.h"

class Command : public Types::TOASTER_TYPES::State_t {
public:
    /**
     * @brief Command state constructor.
     *
     */
    Command(System &system, Types::TOASTER_TYPES::SystemStatus_t& status);

    /**
     * @brief Perform any initialization required for the state
     *
     */
    void initialize() override;

    /**
     * @brief Function called every update cycle, use to implement periodic actions such as checking sensors. If nullptr is returned, the statemachine will loop the state,
     * otherwise pass a new state ptr to transition to a new state.
     *
     * @return std::unique_ptr<State>
     */
    Types::TOASTER_TYPES::State_ptr_t update() override;

    /**
     * @brief Exit state actions, cleanup any files opened, save data that kinda thing.
     *
     */
    void exit() override;

private:
    static double angleErrorFunction(const double target, const double measurement);
    static double angleDerivativeFunction(const double measurement, const double prevMeasurement, const double dt);

    /**
     * @brief Reference to system class
     */
    System& m_system;
    Types::TOASTER_TYPES::SystemStatus_t& m_status;

    /**
     * @brief Torque command controller.
     *
     * Takes in measured roll and a target roll (rad) and produces a requested
     * torque (Nm) as output.
     */
    PID m_pid;

    /// @brief Generates roll setpoints as radian heading angles.
    TargetRollGenerator m_targetRollGenerator;

    // AngleWrapper<AngleWrapperConfig::RADIANS> m_measurementWrapper;
    // AngleWrapper<AngleWrapperConfig::RADIANS> m_setpointWrapper;

    uint64_t m_lastUpdateTimeMs { 0 };
    uint64_t m_lastCommandTimeMs { 0 };
};