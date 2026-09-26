#pragma once
#include <cstdint>
#include <memory>

#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/fsm/state.h>
#include <libriccore/fsm/statemachine.h>
/**
 * @brief Templated struct with type aliases inside to provide convient type access. Some of the template paramters might require
 * forward declaration to prevent cylic dependancies.
 *
 * @tparam TOASTER_FLAGS_T Enum of system flags
 */
enum class TOASTER_FLAGS : uint32_t
{
    // State flags
    STATE_ZERO = (1 << 0),
    STATE_DEFAULT = (1 << 1),
    STATE_ARMED = (1 << 2),
    STATE_DEPLOY = (1 << 3),
    STATE_COMMAND = (1 << 4),

    // Error flags
    ERROR_ZERO_TIMEOUT = (1 << 10)
};

template <typename TOASTER_FLAGS_T>
struct ToasterTypes
{
    using SystemStatus_t = SystemStatus<TOASTER_FLAGS_T>;
    using State_t = State<TOASTER_FLAGS_T>;
    using State_ptr_t = std::unique_ptr<State_t>;
    using StateMachine_t = StateMachine<TOASTER_FLAGS_T>;
};

namespace Types
{
    using TOASTER_TYPES = ToasterTypes<TOASTER_FLAGS>;
};