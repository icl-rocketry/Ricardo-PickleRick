
// #pragma once

// #include <memory>

// #include "Config/systemflags_config.h"
// #include "system.h"

// class Debug : public Types::CoreTypes::State_t
// {
//     public:
//         /**
//          * @brief Debug state constructor. 
//          * 
//          */
//         Debug(System &system);

//         /**
//          * @brief Perform any initialization required for the state
//          * 
//          */
//         void initialize() override;

//         /**
//          * @brief Function called every update cycle, use to implement periodic actions such as checking sensors. If nullptr is returned, the statemachine will loop the state,
//          * otherwise pass a new state ptr to transition to a new state.
//          * 
//          * @return std::unique_ptr<State> 
//          */
//         Types::CoreTypes::State_ptr_t update() override;

//         /**
//          * @brief Exit state actions, cleanup any files opened, save data that kinda thing.
//          * 
//          */
//         void exit() override;

//     private:
//       /**
//        * @brief Reference to system class
//        * 
//        */
//         System &_system;
// };