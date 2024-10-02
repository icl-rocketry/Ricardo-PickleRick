#pragma once

#include <vector>
#include <memory>
#include <unordered_map>

#include <ArduinoJson.h>

#include <libriccore/riccorelogging.h>

#include "event.h"
#include "flightVariables.h"


#include "Deployment/deploymenthandler.h"
#include "Engine/enginehandler.h"

#include "Config/types.h"


class EventHandler{

    public:

        EventHandler(EngineHandler& enginehandler, DeploymentHandler& deploymenthandler, const Types::LocalPyroMap_t& localPyroMap, const Types::LocalServoMap_t& localServoMap):
        _flightvariables(rocketState, *this),
        _enginehandler(enginehandler),
        _deploymenthandler(deploymenthandler),
        m_localPyroMap(localPyroMap),
        m_localServoMap(localServoMap)
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

        /**
         * @brief Reset all events
         * 
         */
        void reset();

        

    private:
        SensorStructs::state_t rocketState;
        
        FlightVariables _flightvariables;

        EngineHandler& _enginehandler;
        DeploymentHandler& _deploymenthandler;

        const Types::LocalPyroMap_t& m_localPyroMap;
        const Types::LocalServoMap_t& m_localServoMap;

        action_t configureAction(JsonVariantConst actions);
        condition_t configureCondition(JsonVariantConst condition,uint8_t recursion_level = 0);
        
        static constexpr uint8_t condition_recursion_max_depth = 6;

        #ifdef _RICDEBUG
        std::string _decisiontree = "";
        #endif
        
        std::vector<std::unique_ptr<Event> > _eventList;

        const std::unordered_map<std::string,std::function<condition_t(EventHandler*, JsonObjectConst conf)>> configureConditionMap = {
                                                                                                                {"flightVar",&EventHandler::configureFlightVarCondition},
                                                                                                                {"localPyroChannel",&EventHandler::configureLocalPyroCondition}
                                                                                                            };

        condition_t configureFlightVarCondition(JsonObjectConst conf);
        condition_t configureLocalPyroCondition(JsonObjectConst conf);


};


