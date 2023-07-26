#pragma once

#include <vector>
#include <memory>

#include <ArduinoJson.h>

#include <libriccore/riccorelogging.h>

#include "event.h"
#include "flightVariables.h"


#include "Deployment/deploymenthandler.h"
#include "Engine/enginehandler.h"


class EventHandler{

    public:

        EventHandler(EngineHandler& enginehandler, DeploymentHandler& deploymenthandler):
        _flightvariables(rocketState, *this),
        _enginehandler(enginehandler),
        _deploymenthandler(deploymenthandler)
        {};

        void setup(JsonArrayConst event_config);

        void update(const SensorStructs::state_t& state);
        /**
         * @brief Get the timestamp when an event was triggered.
         * 
         * @param eventID 
         * @return timestamp in milliseconds. Returns 0 if has not been triggered yet
         */
        uint32_t timeTriggered(uint8_t eventID);

        

    private:
        SensorStructs::state_t rocketState;
        
        FlightVariables _flightvariables;

        EngineHandler& _enginehandler;
        DeploymentHandler& _deploymenthandler;

        action_t configureAction(JsonVariantConst actions);
        condition_t configureCondition(JsonVariantConst condition,uint8_t recursion_level = 0);
        
        static constexpr uint8_t condition_recursion_max_depth = 6;

        #ifdef _RICDEBUG
        std::string _decisiontree = "";
        #endif
        
        std::vector<std::unique_ptr<Event> > _eventList;
};


