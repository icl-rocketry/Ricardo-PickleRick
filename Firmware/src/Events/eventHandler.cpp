#include "eventHandler.h"

#include <ArduinoJson.h>
#include <stdexcept>
#include <memory>

#include <libriccore/riccorelogging.h>
#include <librrc/Helpers/jsonconfighelper.h>
#include <librrc/Local/remoteactuatoradapter.h>
#include <librnp/default_packets/simplecommandpacket.h>

#include "Config/services_config.h"

#include "event.h"
#include "condition.h"
#include "flightVariables.h"




void EventHandler::setup(JsonArrayConst event_config)
{

    using namespace LIBRRC::JsonConfigHelper;
    //for each in the list of cool tings

    if (event_config.isNull())
    {
        return; // dont do nothing we have nothin to do
    }

    size_t event_list_size = event_config.size();
    if (event_list_size == 0){
         RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("EventHandler no events provided!");
        return;
    }

    _eventList.resize(event_list_size); // allocate elements

    int eventID = 0;

    for (JsonObjectConst jsonEvent : event_config)
    {
        #ifdef _RICDEBUG //TODO cleanup this
        _decisiontree = "";
        #endif
        //verify action ID 
        checkConfigId(eventID,jsonEvent);

        std::string eventName = getIfContains<std::string>(jsonEvent,"name",std::string("Unnamed"));
    
        // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(eventName);
        bool fire_mode = getIfContains<bool>(jsonEvent,"single_fire",true); //default true

        uint16_t actionCooldown = getIfContains<uint16_t>(jsonEvent,"cooldown",1000); //default 1 second cooldown

        //! Null configs for the condition config and the action config are handleld in their respective functions
        JsonVariantConst conditionJson = jsonEvent["condition"];
        condition_t eventCondition = configureCondition(conditionJson);

        JsonVariantConst actionJson = jsonEvent["action"];
        action_t eventAction;
    
        if (actionJson.isNull()){ //either it doesnt exist or "action":null
            eventAction = [](){}; // null action
        }else{
            eventAction = configureAction(actionJson);
        }

        _eventList.at(eventID) = std::make_unique<Event>(eventID,
                                                 eventName,
                                                 eventCondition,
                                                 eventAction,
                                                 fire_mode,
                                                 actionCooldown);

        #ifdef _RICDEBUG //TODO CLEANUP
         RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(_decisiontree);
        #endif

        eventID++; //increment id
    }
};


action_t EventHandler::configureAction(JsonVariantConst actions){

    if (actions.is<JsonObjectConst>()){

        auto actionJson = actions.as<JsonObjectConst>();
        std::string actionType = actionJson["type"];
        
    
        if (actionType == "engine")
        {
            int actionID = actionJson["id"];
            int actionParam = actionJson["param"];
            return [actionParam,actionID,this](){_enginehandler.getActionFunc(actionID)(actionParam);}; 
        }
        else if (actionType == "deployment")
        {
            int actionID = actionJson["id"];
            int actionParam = actionJson["param"];
            return [actionParam,actionID,this](){_deploymenthandler.getActionFunc(actionID)(actionParam);};
        }
        //TODO add generic command functor
        else if (actionType == "command")
        {   
            uint8_t source = m_networkmanager.getAddress();
            uint8_t source_service = static_cast<uint8_t>(Services::ID::EventHandler);

            uint8_t destination = actions["destination"];
            uint8_t destination_service = actions["destination_service"];
            uint8_t command_id = actions["command_id"];
            int32_t command_arg = actions["command_arg"];

            //consturct command packet
            SimpleCommandPacket command_packet(command_id,command_arg);
            command_packet.header.source = source;
            command_packet.header.source_service = source_service;
            command_packet.header.destination = destination;
            command_packet.header.destination_service = destination_service;
            command_packet.command = command_id;
            command_packet.arg = command_arg;

            auto commandFunctor = [this,command_packet]()
            {
                SimpleCommandPacket command = command_packet;
                this->m_networkmanager.sendPacket(command);
            };

            return commandFunctor;
        }
        else if (actionType == "null")
        {
            return [](){};
        }
        else
        {
            #ifdef _RICDEBUG
            std::string actionJsonSer;
            serializeJson(actionJson,actionJsonSer);
            throw std::runtime_error("Bad Action Type: " + actionJsonSer);
            #endif
             RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Bad Action Type, continuing with null action");
            return [](){};
        }
    }else if (actions.is<JsonArrayConst>()) {
        std::vector<action_t> action_vec;

        for (JsonObjectConst action : actions.as<JsonArrayConst>()) {
            action_vec.push_back(configureAction(action));
        }

        return [action_vec = std::move(action_vec)]() {
            for (auto& action : action_vec) {
                action();
            }
        };
    }else{
        #ifdef _RICDEBUG
        std::string actionJsonSer;
        serializeJson(actions,actionJsonSer);
        throw std::runtime_error("Bad Action config: " + actionJsonSer);
        #endif
        // return null i.e no action
         RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Bad Action config, continuing with null action");
        return [](){}; // null action
    }
}

condition_t EventHandler::configureCondition(JsonVariantConst condition, uint8_t recursion_level)
{
    if (recursion_level > condition_recursion_max_depth){
        throw std::runtime_error("EventHandler max recursion depth reached!"); // this needs to fail quietly too
    }

    if (!condition.is<JsonObjectConst>())
    {
        throw std::runtime_error("EventHandler Invalid type deserialized : " + condition.as<std::string>() + " depth : " + std::to_string(recursion_level));
    }

    auto conditionJson = condition.as<JsonObjectConst>();


    if (conditionJson.containsKey("condition")){

        auto subconditionarray = conditionJson["condition"].as<JsonArrayConst>(); // required explict cast to JsonArray type
        size_t arraysize = subconditionarray.size();

        if (arraysize == 0){
            throw std::runtime_error("EventHandler no conditions provided"); // we can fail safe here by returning false
        }
        if (arraysize == 1){
            return configureCondition(subconditionarray[0],recursion_level); // no change to recursion depth
        }

        conditionOperator_t op;
        
        if (conditionJson["operator"].as<std::string>() == "AND"){
            op = ConditionOperator::AND;
        }else if (conditionJson["operator"].as<std::string>() == "OR"){
            op = ConditionOperator::OR;
        }else{
            throw std::runtime_error("Invalid Operator Supplied");
        }
        
        //generate decision tree

        //get initial condition
        #ifdef _RICDEBUG
        _decisiontree += "  (";
        #endif

        auto conditionCombination = configureCondition(subconditionarray[0],recursion_level + 1);
        
        for (int i = 1; i < arraysize;i++){
            #ifdef _RICDEBUG
            _decisiontree += "  ";
            _decisiontree += conditionJson["operator"].as<std::string>();
            _decisiontree += "  ";
            #endif

            conditionCombination = ConditionCombination(conditionCombination,
                                                            configureCondition(subconditionarray[i],recursion_level + 1),
                                                            op);
            #ifdef _RICDEBUG
            _decisiontree += " ) ";
            #endif

        }
        

        return conditionCombination;

    }
    

    //iterate thru call map 
    //!O(n) but this doesnt matter right now, could redo to use explict keys in JSON and use that
    //! to lookup in map but thats a future thing.
    for (const auto& [key, configureConditionFunction] : configureConditionMap)
    {
        if (conditionJson[key])
        {
            return configureConditionFunction(this,conditionJson);
        }
    }
    
    
    //fall through

    std::string condition_string;
    serializeJson(conditionJson,condition_string);
    throw std::runtime_error("EventHandler Bad Condition Type " + condition_string);



    //networkcomponentstate
    //networkcomponentvalue
    //command id recieved for manual triggers
    //pyro continuity

}

condition_t EventHandler::configureFlightVarCondition(JsonObjectConst conf)
{
    // what happens if config requests that flight var is time since event on its self?
        // if it is the only condition and u are testing time_triggered then the event will never trigger
        // if the conditon is 'and' with another condition it will again never trigger
        // if the conditon is or'ed then if the other condition is met, the event will trigger. If the conditon comparing
        // time trigered is less than, the result is no different to just testing the other conditon. If using more than, 
        // we effectivley latch the output of the event, it will always be on meaning the action will be continually called
        
        flightVariableOperator_t op;
        if (conf["operator"].as<std::string>() == "LESSTHAN"){
            op = ConditionOperator::LESSTHAN;
        }else if (conf["operator"].as<std::string>() == "MORETHAN"){
            op = ConditionOperator::MORETHAN;
        }else{
            throw std::runtime_error("Invalid Operator Supplied");
        }

        #ifdef _RICDEBUG
        serializeJson(conf,_decisiontree);
        #endif

        return Condition(_flightvariables.get(conf["flightVar"].as<std::string>()), 
                            conf["component"].as<int>(),
                            conf["threshold"].as<float>(),
                            op); // while this looks like we are returning an Condtion object, as the return type of the function is std::function<bool()> and the operator() is defined for condtion, we are in fact returning a callable function

}

condition_t EventHandler::configureLocalPyroCondition(JsonObjectConst conf)
{
    if (!conf["continuity"])
    {
        throw std::runtime_error("EventHandler Condition no state given ");
    }

    auto localPyroChannel = conf["localPyroChannel"].as<int>();
    //check if channel exists
    if (localPyroChannel > m_localPyroMap.size())
    {
        throw std::runtime_error("EventHandler Condition localPyroChannel out of range");
    }

    bool continuity = conf["continuity"].as<bool>();

    auto condtionFunc = [localPyroChannel,continuity,this]() -> bool {
        auto localPyro = RemoteActuatorAdapter<Types::LocalPyro_t>(0,*m_localPyroMap.at(localPyroChannel),[](std::string msg){});
        localPyro.updateState(); // force pyro to read continuity
        auto state = localPyro.getState();
        if (!state.flagSet(LIBRRC::COMPONENT_STATUS_FLAGS::ERROR_CONTINUITY) && continuity)
        {
            return true; 
        }
        return false; 
    };

    return condtionFunc;
}


void EventHandler::update(const SensorStructs::state_t& state)
{
    //copy rocket state to private rocket state -> idea behind this is in the future 
    //copy allows event updates to happen in a different task to the estimator
    rocketState = state;

    for (auto &event : _eventList)
    {
        event->update();
    }

};

uint32_t EventHandler::timeTriggered(uint8_t eventID)
{
    try
    {
        return _eventList.at(eventID)->timeTriggered();
    }
    catch (std::out_of_range const &)
    {
        return 0; //this is bad
    }

}

void EventHandler::reset(){
    for (auto& event : _eventList)
    {
        event->reset();
    }
}

