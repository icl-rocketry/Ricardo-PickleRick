
#pragma once

#include <memory>

#include "Toaster/types.h"
#include "system.h"

class Armed : public Types::TOASTER_TYPES::State_t
{
    public:
        /**
         * @brief Armed state constructor.
         *
         */
        Armed(System &system, Types::TOASTER_TYPES::SystemStatus_t& status);

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
      /**
       * @brief Reference to system class
       *
       */
        System& m_system;
        Types::TOASTER_TYPES::SystemStatus_t& m_status;
};