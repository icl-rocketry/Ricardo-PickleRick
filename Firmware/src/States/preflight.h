/*
State when rocket is not in flight. Allows resetting and intilization of all sensors and check all snesors for anomalies. 
Allows flash system operation to dump/reformat of onboard and sd card.
If serial input detected switches to usb mode.
Transtion to countdown state happens through lora command
*/

#pragma once

#include <memory>

#include <libriccore/fsm/state.h>

#include "system.h"

#include "Config/systemflags_config.h"
#include "Config/types.h"

class Preflight : public Types::CoreTypes::State_t
{
    public:
        /**
         * @brief Preflight state constructor. All states require the systemstatus object to be passed in, as well as any other system level objects required. For example, if
         * we want to control the available commands, we need to pass in the command handler from the riccoresystem.
         * 
         */
        Preflight(System& system);

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
        System& _system;
};