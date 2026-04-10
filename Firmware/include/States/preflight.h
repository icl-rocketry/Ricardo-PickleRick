#pragma once

#include <memory>

#include <libriccore/fsm/state.h>

#include "system.h"

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include "Config/commands_config.h"

#include "States/flight.h"

class Preflight : public Types::CoreTypes::State_t
{
    public:
        Preflight(System& system);

        void initialize() override;

        Types::CoreTypes::State_ptr_t update() override;

        void exit() override;

    private:
        System& _system;

};