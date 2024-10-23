#pragma once
// object representing event
#include <memory>
#include <functional>
#include <string>

#include "condition.h"

#include <libriccore/riccorelogging.h>

using condition_t = std::function<bool()>;
using action_t = std::function<void()>;

class Event
{
public:
    Event(int eventID, std::string name, condition_t condition, action_t action, bool singleUse, uint16_t actionCooldown) : _eventID(eventID),
                                                                                                                            _name(name),
                                                                                                                            _condition(std::move(condition)),
                                                                                                                            _action(std::move(action)),
                                                                                                                            _singleUse(singleUse),
                                                                                                                            _cooldown(actionCooldown),
                                                                                                                            _timeTriggered(0), // initialize to zero as no event can be triggered at 'zero' time
                                                                                                                            _lastActionTime(0),
                                                                                                                            _previouslyFired(false)
    {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Event " + std::to_string(_eventID) + " : " + _name + " created");
    };

    void update();

    uint32_t timeTriggered() { return _timeTriggered; };

    void reset();

private:
    /**
     * @brief Event identifier
     *
     */
    const int _eventID;
    /**
     * @brief Event readable name
     *
     */
    std::string _name;
    /**
     * @brief condition functor to check if event should be triggered
     *
     */
    const condition_t _condition;
    /**
     * @brief Event action functor, exectued when event is triggered
     *
     */
    const action_t _action;
    /**
     * @brief Can the event be fired multiple times
     *
     */
    const bool _singleUse;
    /**
     * @brief Delay between sucssesive event triggering, if _singleFire is false
     *
     */
    const uint16_t _cooldown;

    /**
     * @brief Time in millis when event was triggered
     *
     */
    uint32_t _timeTriggered;
    /**
     * @brief Time in millis when the event was triggered previously
     *
     */
    uint32_t _lastActionTime;

    /**
     * @brief True if the event has been previously fired
     *
     */
    bool _previouslyFired;
};
