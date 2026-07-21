#pragma once

#include <array>
#include <cstddef>
#include <memory>

#include <libriccore/fsm/state.h>
#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include "Config/commands_config.h"
#include "Config/flight_trajectory_config.h"

#include "States/landing.h"
#include "Trajectory/TrapezoidalTrajectory.h"
#include "system.h"

class Flight : public Types::CoreTypes::State_t
{
    public:

        Flight(System& system);

        void initialize() override;

        Types::CoreTypes::State_ptr_t update() override;

        void exit() override;

    private:
        bool configureTrajectoryLeg(std::size_t leg_index);

        System& _system;
        Trajectory::TrapezoidalTrajectory m_position_trajectory;
        uint32_t m_trajectory_start_ms = 0;
        bool m_trajectory_active = true;
        bool m_tilt_power_cut = false;
        std::size_t m_trajectory_leg_index = 0;
};
