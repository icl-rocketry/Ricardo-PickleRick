#include "flightVariables.h"

#include <Arduino.h>
#include <optional>
#include <variant>
#include <functional>
#include <unordered_map>

#include "eventHandler.h"

const std::unordered_map<std::string, memberFunc_t> FlightVariables::function_map {
	{"TimeSinceEvent", &FlightVariables::TimeSinceEvent},
	{"Position", &FlightVariables::Position},
	{"Velocity", &FlightVariables::Velocity},
	{"Acceleration", &FlightVariables::Acceleration}
	//networkcomponentstate
	//networkcomponentvalue
	//pyro continuity
};

flightVariable_t FlightVariables::TimeSinceEvent(int arg) 
{
	uint32_t eventTime = _eventhandler.timeTriggered(arg);
	return timeSince(eventTime);
}

flightVariable_t FlightVariables::Position(int arg) 
{
	return getComponent(_state.position,arg);
}

flightVariable_t FlightVariables::Velocity(int arg) 
{
	return getComponent(_state.velocity, arg);
}

flightVariable_t FlightVariables::Acceleration(int arg) 
{
	return getComponent(_state.acceleration, arg);
}


flightVariableFunc_t FlightVariables::get(const std::string& funcName) {
	auto funcptr = function_map.at(funcName);
	return [this,funcName,funcptr](int idx) {
		flightVariable_t var = ((*this).*(funcptr))(idx);
		#ifdef _RICDEBUG
		const std::string variablename(funcName);
		std::cout<<"[FlightVar] :"+variablename+" called with value : " + std::to_string(var.value_or(0))<<std::endl;
		#endif
		return var;
	};
}

std::optional<float> FlightVariables::timeSince(const uint32_t time) {
	if (!time) {
		return {};
	}

	uint32_t currTime = millis();
	if (currTime < time) {
		return {};
	}

	return {static_cast<float>(currTime - time)};

}

std::optional<float> FlightVariables::getComponent(const Eigen::Vector3f& var, int arg) {
	if (arg == -1) {
		return {var.norm()};
	} else if (arg < -1 || arg >= var.size()) {
		#ifdef _RICDEBUG
			throw std::runtime_error("Chorley fucked the config!");
		#endif
		return {};
	}
	return {var(arg)};
}
