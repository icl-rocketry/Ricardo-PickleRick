#pragma once

#include <array>
#include <cstdint>
#include <memory>

#include <libriccore/fsm/state.h>
#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include "Config/commands_config.h"

#include "Trajectory/TrapezoidalTrajectory.h"
#include "system.h"

class Landing : public Types::CoreTypes::State_t
{
    public:

        Landing(System& system);

        void initialize() override;

        Types::CoreTypes::State_ptr_t update() override;

        void exit() override;

    private:
        enum class Phase {
            Descent,
            ThrottleFade,
            Complete,
        };

        void beginThrottleFade();
        void updateThrottleFade(uint32_t current_time_ms);
        void stopControllerOnce();
        bool reachedCutoffHeight(const Eigen::Vector3f& position_ned) const;

        System& _system;
        Trajectory::TrapezoidalTrajectory m_descent_trajectory;
        Eigen::Vector3f m_target_position_ned = Eigen::Vector3f::Zero();
        Eigen::Vector4f m_fade_start_output = Eigen::Vector4f::Zero();
        uint32_t m_phase_start_ms = 0;
        Phase m_phase = Phase::Descent;
        bool m_descent_active = false;
        bool m_controller_stopped = false;
};
