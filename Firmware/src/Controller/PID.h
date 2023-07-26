#pragma once

#include "controller.h"
#include "controllable.h"

#include <libriccore/riccorelogging.h>

// #include "Storage/logController.h"



//purley an example of a control system. Controllers should be implemented more specifically 
//as generalization here is not ideal-> leads to alot mroe overhead and isnt worth it
class PID : public Controller {
public:
	PID(uint8_t id,float Kp, float Ki, float Kd,float setpoint,Controllable* doohickey, uint32_t update_interval = 0):
		Controller(id,doohickey, logcontroller, update_interval),
		_Kp(Kp),
		_Ki(Ki),
		_Kd(Kd),
		_setpoint(setpoint)	
	{
		RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("[ERROR] PID EXAMPLE CS CONSTRUCTED, DONT USE IN LIVE SYSTEM");
		RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("PID CS:" + std::to_string(_id) + " constructed with Kp:" + std::to_string(_Kp) + " Ki:" + std::to_string(_Ki) + " Kd:" + std::to_string(_Kd) + " Setpoint:" + std::to_string(setpoint) + " update_interval:" + std::to_string(_update_interval));
		
	};

	/**
	 * @brief Very naive implementation of first order discrete PID
	 *
	 */
	inline void calculate(const SensorStructs::state_t& estimator_state) override
	{
		float input = estimator_state.position[1];
		float error = _setpoint - input;
		float derror = (error - prevError) / static_cast<float>(deltaT);
		intError += (error * static_cast<float>(deltaT));

		float output = (_Kp * error) + (_Ki * intError) + (_Kd * derror);
		// RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("CS:" + std::to_string(_id) + " output: " + std::to_string(output));
		_controllable->control({output});
	};

private:
	float _Kp, _Ki, _Kd; 
	
	float _setpoint;

	float prevError;
	float intError;

	
};