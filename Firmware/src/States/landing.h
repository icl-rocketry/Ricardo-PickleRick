#pragma once


#include <array>
#include <memory>

#include <libriccore/fsm/state.h>
#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/commands/commandhandler.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"

#include "system.h"
#include "hardAbort.h"
#include "softAbort.h"


class Landing : public Types::CoreTypes::State_t
{
    public:
        Landing(System& system);

        void initialize() override;

        Types::CoreTypes::State_ptr_t update() override;

        void exit() override;

    private:
        System& _system;
        
};