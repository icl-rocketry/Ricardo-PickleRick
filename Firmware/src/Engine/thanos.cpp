#include "thanos.h"
#include "Helpers/jsonconfighelper.h"

Thanos::Thanos(uint8_t id, JsonObjectConst engineConfig, addNetworkCallbackFunction_t addNetworkCallbackFunction, RnpNetworkManager &networkmanager, uint8_t handlerServiceID) : 
Engine(id, networkmanager, handlerServiceID)
{
    using namespace JsonConfigHelper;

    auto engineConf = getIfContains<JsonObjectConst>(engineConfig, "engine");
    _engine = std::make_unique<NetworkActuator>(0,
                                                 getIfContains<uint8_t>(engineConf, "address"),
                                                 handlerServiceID,
                                                 getIfContains<uint8_t>(engineConf, "destination_service"),
                                                 _networkmanager,
                                                 getLogCB());
    
    addComponentNetworkCallback(_engine.get(),engineConf,addNetworkCallbackFunction);

    auto igniterConf = getIfContains<JsonObjectConst>(engineConfig, "igniter");
    _igniter = std::make_unique<NetworkActuator>(1,
                                                 getIfContains<uint8_t>(igniterConf, "address"),
                                                 handlerServiceID,
                                                 getIfContains<uint8_t>(igniterConf, "destination_service"),
                                                 _networkmanager,
                                                 getLogCB());   

    addComponentNetworkCallback(_igniter.get(),igniterConf,addNetworkCallbackFunction);

    auto oxVentValveConf = getIfContains<JsonObjectConst>(engineConfig, "oxVentValve");
    _oxVentValve = std::make_unique<NetworkActuator>(2,
                                                getIfContains<uint8_t>(oxVentValveConf, "address"),
                                                handlerServiceID,
                                                getIfContains<uint8_t>(oxVentValveConf, "destination_service"),
                                                _networkmanager,
                                                getLogCB());

    setIfContains<uint16_t>(oxVentValveConf,"closed_position",_oxVentValveClosed,false);
    setIfContains<uint16_t>(oxVentValveConf,"open_position",_oxVentValveOpen,false);

    addComponentNetworkCallback(_oxVentValve.get(),oxVentValveConf,addNetworkCallbackFunction);

    auto fuelVentValveConf = getIfContains<JsonObjectConst>(engineConfig, "fuelVentValve");
    _fuelVentValve = std::make_unique<NetworkActuator>(3,
                                                getIfContains<uint8_t>(fuelVentValveConf, "address"),
                                                handlerServiceID,
                                                getIfContains<uint8_t>(fuelVentValveConf, "destination_service"),
                                                _networkmanager,
                                                getLogCB());

    setIfContains<uint16_t>(fuelVentValveConf,"closed_position",_fuelVentValveClosed,false);
    setIfContains<uint16_t>(fuelVentValveConf,"open_position",_fuelVentValveOpen,false);

    addComponentNetworkCallback(_fuelVentValve.get(),fuelVentValveConf,addNetworkCallbackFunction);

    auto fuelPrssValveConf = getIfContains<JsonObjectConst>(engineConfig, "fuelPrssValve");
    _fuelPrssValve = std::make_unique<NetworkActuator>(4,
                                                getIfContains<uint8_t>(fuelPrssValveConf, "address"),
                                                handlerServiceID,
                                                getIfContains<uint8_t>(fuelPrssValveConf, "destination_service"),
                                                _networkmanager,
                                                getLogCB());

    setIfContains<uint16_t>(fuelPrssValveConf,"closed_position",_fuelPrssValveClosed,false);
    setIfContains<uint16_t>(fuelPrssValveConf,"open_position",_fuelPrssValveOpen,false);

    addComponentNetworkCallback(_fuelPrssValve.get(),fuelPrssValveConf,addNetworkCallbackFunction);
    log("Thanos Constructed");
}

void Thanos::updateState()
{
    _engine->updateState();
    _igniter->updateState();
    _oxVentValve->updateState();
    _fuelVentValve->updateState();
    _fuelPrssValve->updateState();
}

void Thanos::execute(int32_t func)
{
    Engine::execute(func);//add vent here
}

void Thanos::armEngine()
{
    _engine->arm();
    _igniter->arm();
    _oxVentValve->arm();
    _fuelVentValve->arm();
    _fuelPrssValve->arm();
}

void Thanos::shutdown()
{
    _engine->execute(shutdown_command_arg);
    Engine::shutdown();
    _state.runState = static_cast<uint8_t>(ENGINE_RUN_STATE::SHUTDOWN);
}

void Thanos::ignite()
{
    Engine::ignite();
    _state.runState = static_cast<uint8_t>(ENGINE_RUN_STATE::IGNITION);
    _engine->execute(ignition_command_arg);
}

void Thanos::update()
{

    if(_state.runState == static_cast<uint8_t>(ENGINE_RUN_STATE::SHUTDOWN)){
        if(millis() - getStatePtr()->shutdownTime > vent_delay){
            _oxVentValve->execute(_oxVentValveOpen);
            _fuelVentValve->execute(_fuelVentValveOpen);
            _fuelPrssValve->execute(_fuelPrssValveOpen);
        } 
    }
}

uint8_t Thanos::flightCheck()
{
    uint8_t res = 0;

    res += _engine->flightCheck(_networkRetryInterval,_componentStateExpiry,"Engine:" + std::to_string(getID()));
    res += _igniter->flightCheck(_networkRetryInterval,_componentStateExpiry,"Engine:" + std::to_string(getID()));
    res += _oxVentValve->flightCheck(_networkRetryInterval,_componentStateExpiry,"Engine:" + std::to_string(getID()));
    res += _fuelVentValve->flightCheck(_networkRetryInterval,_componentStateExpiry,"Engine:" + std::to_string(getID()));
    res += _fuelPrssValve->flightCheck(_networkRetryInterval,_componentStateExpiry,"Engine:" + std::to_string(getID()));
    
    return res;
    
}

void Thanos::control(std::vector<float> u){};