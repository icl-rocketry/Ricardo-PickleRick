#include "event.h"

#include <string>

#include <Arduino.h>


#include <libriccore/riccorelogging.h>

void Event::update()
{
	if (_singleUse && _previouslyFired)
	{
		return; //no need to continue evaluating the contiions
	}

	if (_condition())
	{ //check if condition has been met

		//update time triggered
		_timeTriggered = millis();

		//update _previouslyFired flag
		_previouslyFired = true;

		// execute action
		if (millis() - _lastActionTime > _cooldown)
		{
			_action();
			_lastActionTime = millis();
			 RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Event:" + std::to_string(_eventID) + " fired at " + std::to_string(_timeTriggered));
		}
	}
}
