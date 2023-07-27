/*
State which deploys chutes after rocket has reached apogee.
*/



#pragma once

#include <memory>

#include <libriccore/fsm/state.h>
#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/commands/commandhandler.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"

class Recovery : public Types::CoreTypes::State_t
{
    public:
        /**
         * @brief Recovery state constructor. All states require the systemstatus object to be passed in, as well as any other system level objects required. For example, if
         * we want to control the available commands, we need to pass in the command handler from the riccoresystem.
         * 
         */
        Recovery(Types::CoreTypes::SystemStatus_t& systemtatus, Types::CoreTypes::CommandHandler_t& commandhandler);

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
        Types::CoreTypes::State_ptr_t update() override;

        /**
         * @brief Exit state actions, cleanup any files opened, save data that kinda thing.
         * 
         */
        void exit() override;

    private:
        Types::CoreTypes::CommandHandler_t& _commandhandler;
};