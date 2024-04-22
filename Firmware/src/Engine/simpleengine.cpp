#include "simpleengine.h"

#include <memory>

#include <ArduinoJson.h>

#include <librrc/Interface/networkactuator.h>
#include <librrc/Handler/configurabledynamichandler.h>
#include <librrc/componentstatusflags.h>
#include <librrc/Local/remoteactuatoradapter.h>


#include <librrc/Helpers/jsonconfighelper.h>

#include "Config/types.h"

SimpleEngine::SimpleEngine(uint8_t id,JsonObjectConst engineConfig,const Types::LocalPyroMap_t &localPyroMap,addNetworkCallbackFunction_t addNetworkCallbackFunction,RnpNetworkManager& networkmanager,uint8_t handlerServiceID):
Engine(id,networkmanager,handlerServiceID),
_state({static_cast<uint8_t>(ENGINE_RUN_STATE::SHUTDOWN),})
{
    using namespace LIBRRC::JsonConfigHelper;
    auto igniterConf = getIfContains<JsonObjectConst>(engineConfig,"igniter");
    auto igniterType = getIfContains<std::string>(igniterConf,"type");

    _igniterParam = getIfContains<uint8_t>(igniterConf,"param");

    if (igniterType == "local_pyro")
    {
        uint8_t channel = getIfContains<uint8_t>(igniterConf, "channel");
        if (channel > 3)
        {
            throw std::runtime_error("Local pyro channel out of range!");
        }
        // retrive nrcremotepyro instance correspondign to channel number
        Types::LocalPyro_t &localPyro = *(localPyroMap.at(channel));

        // add object to dep handler and use adapter to convert to local type
        _igniter = std::make_unique<RemoteActuatorAdapter<Types::LocalPyro_t>>(id, localPyro, getLogCB());
        
    }
    else if (igniterType == "net_actuator")
    {
        auto igniterAddress = getIfContains<uint8_t>(igniterConf, "address");
        auto igniterDestinationService = getIfContains<uint8_t>(igniterConf, "destination_service");
        _igniter = std::make_unique<NetworkActuator>(0,
                                                     igniterAddress,
                                                     _handlerServiceID,
                                                     igniterDestinationService,
                                                     networkmanager,
                                                     getLogCB());
        addNetworkCallbackFunction(
            igniterAddress,
            igniterDestinationService,
            [this](packetptr_t packetptr)
            {
                dynamic_cast<NetworkActuator *>(_igniter.get())->networkCallback(std::move(packetptr));
            },
            true);
    }
    else
    {
        throw std::runtime_error("Invalid igniter type!");
    }
};
        

void SimpleEngine::updateState(){
    _igniter->updateState();
}

uint8_t SimpleEngine::flightCheck(){
    uint8_t flightcheck_result = _igniter->flightCheck(_networkRetryInterval,_componentStateExpiry,"SimpleEngine");
    if (_igniter->getState().flagSet(LIBRRC::COMPONENT_STATUS_FLAGS::ERROR_NORESPONSE) ){
        _state.connectionState = static_cast<uint8_t>(ENGINE_CONNECTION_STATE::ERROR);
    }else{
        _state.connectionState = static_cast<uint8_t>(ENGINE_CONNECTION_STATE::CONNECTED);
    }
    return flightcheck_result;
}

void SimpleEngine::ignite(){
    Engine::ignite();
    _igniter->execute(_igniterParam); // fire the pyro let it rip
    _state.runState = static_cast<uint8_t>(ENGINE_RUN_STATE::RUNNING);// goes straight to running state as there is no igntion time
}

void SimpleEngine::shutdown(){ // the engine cant be shut down lol
    Engine::shutdown();
    _state.runState = static_cast<uint8_t>(ENGINE_RUN_STATE::SHUTDOWN);
}

void SimpleEngine::armEngine(){
    _igniter->arm();
}

