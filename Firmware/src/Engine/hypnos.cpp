#include "hypnos.h"
#include <librrc/Helpers/jsonconfighelper.h>

Hypnos::Hypnos(uint8_t id, JsonObjectConst engineConfig, addNetworkCallbackFunction_t addNetworkCallbackFunction, RnpNetworkManager &networkmanager, uint8_t handlerServiceID) : 
Engine(id, networkmanager, handlerServiceID),
_igniterFired(false),
motor_lockout(true),
shutdown_called(false)
{
    using namespace LIBRRC::JsonConfigHelper;
    //setup components from config

    auto igniterConf = getIfContains<JsonObjectConst>(engineConfig, "igniter");
    _igniter = std::make_unique<NetworkActuator>(0,
                                                 getIfContains<uint8_t>(igniterConf, "address"),
                                                 handlerServiceID,
                                                 getIfContains<uint8_t>(igniterConf, "destination_service"),
                                                 _networkmanager,
                                                 getLogCB());

    setIfContains<uint16_t>(igniterConf,"fire_duration",_igniterFiringTime,false);

    addComponentNetworkCallback(_igniter.get(),igniterConf,addNetworkCallbackFunction);

    auto oxidiserValveConf = getIfContains<JsonObjectConst>(engineConfig, "oxidiserValve");
    _oxidiserValve = std::make_unique<NetworkActuator>(1,
                                                 getIfContains<uint8_t>(oxidiserValveConf, "address"),
                                                 handlerServiceID,
                                                 getIfContains<uint8_t>(oxidiserValveConf, "destination_service"),
                                                 _networkmanager,
                                                 getLogCB());

    setIfContains<uint16_t>(oxidiserValveConf,"closed_position",_oxidiserValveClosed,false);
    setIfContains<uint16_t>(oxidiserValveConf,"open_position",_oxidiserValveOpen,false);
    setIfContains<uint16_t>(oxidiserValveConf,"preignition_position",_oxidiserValvePreIgnition,false);

    addComponentNetworkCallback(_oxidiserValve.get(),oxidiserValveConf,addNetworkCallbackFunction);

    auto ventValveConf = getIfContains<JsonObjectConst>(engineConfig, "ventValve");
    _ventValve = std::make_unique<NetworkActuator>(2,
                                                 getIfContains<uint8_t>(ventValveConf, "address"),
                                                 handlerServiceID,
                                                 getIfContains<uint8_t>(ventValveConf, "destination_service"),
                                                 _networkmanager,
                                                 getLogCB());
                                                 
    setIfContains<uint16_t>(ventValveConf,"closed_position",_ventValveClosed,false);
    setIfContains<uint16_t>(ventValveConf,"open_position",_ventValveOpen,false);

    addComponentNetworkCallback(_ventValve.get(),ventValveConf,addNetworkCallbackFunction);
   
    //SENSOR SETUP

    auto chamberPressureConf = getIfContains<JsonObjectConst>(engineConfig, "chamberPressure");
    _chamberPressure = std::make_unique<NetworkSensor>(3,
                                                 getIfContains<uint8_t>(chamberPressureConf, "address"),
                                                 handlerServiceID,
                                                 getIfContains<uint8_t>(chamberPressureConf, "destination_service"),
                                                 _networkmanager,
                                                 getLogCB());
                                                 
    addComponentNetworkCallback(_chamberPressure.get(),chamberPressureConf,addNetworkCallbackFunction);

    auto tankPressureConf = getIfContains<JsonObjectConst>(engineConfig, "tankPressure");
    _tankPressure = std::make_unique<NetworkSensor>(4,
                                                 getIfContains<uint8_t>(tankPressureConf, "address"),
                                                 handlerServiceID,
                                                 getIfContains<uint8_t>(tankPressureConf, "destination_service"),
                                                 _networkmanager,
                                                 getLogCB());
                                                 
    addComponentNetworkCallback(_tankPressure.get(),tankPressureConf,addNetworkCallbackFunction);

    log("Hypnos Constructed");

}

void Hypnos::updateState(){//force update all components
    _igniter->updateState();
    _oxidiserValve->updateState();
    _ventValve->updateState();
    _chamberPressure->updateState();
    _tankPressure->updateState();
}

void Hypnos::execute(int32_t func){
    switch(func){
        case static_cast<uint8_t>(HYPNOS_EXECTUE::VENT):
        {
            // Kiran is a poo
            vent();
            break;
        }
        default:
        {
            Engine::execute(func);
            break;
        }
    }
}

void Hypnos::vent() {
    log("Venting Started");
    _oxidiserValve->execute(_oxidiserValveClosed);
    _ventValve->execute(_ventValveOpen);
}

void Hypnos::armEngine(){
    _ventValve->arm();
    _oxidiserValve->arm();
    _igniter->arm();
};

void Hypnos::shutdown(){
    shutdown_called=true;
    if (motor_lockout){
        log("shutdown called during motor lockout");
        return;
    }
    Engine::shutdown();
    _oxidiserValve->execute(_oxidiserValveClosed); // close main ox valve
    _ventValve->execute(_ventValveOpen);
    _state.runState = static_cast<uint8_t>(ENGINE_RUN_STATE::SHUTDOWN);
}

void Hypnos::ignite(){
    Engine::ignite();
    _state.runState = static_cast<uint8_t>(ENGINE_RUN_STATE::IGNITION);
    _ventValve->execute(_ventValveClosed);
    log("oxidiser valve to pre-ignition");
    _oxidiserValve->execute(_oxidiserValvePreIgnition);
}

void Hypnos::update(){
    
    updateSensors();
    heartbeat();

    if (_state.runState == static_cast<uint8_t>(ENGINE_RUN_STATE::IGNITION)){//if we are still in ignition phase
        if ((millis() - _state.ignitionTime > _preIgnitionDelay ) && (!_igniterFired)){ // ignite igniter
            log("firing igniter");
            _igniter->execute(_igniterFiringTime);
            _igniterFired = true;
        }else if (millis() - _state.ignitionTime > _postIgnitionDelay+_preIgnitionDelay){
            log("oxidiser valve to fully open");
            _oxidiserValve->execute(_oxidiserValveOpen);
            _state.runState = static_cast<uint8_t>(ENGINE_RUN_STATE::RUNNING);
        }
    }

    if ((millis()-_state.ignitionTime > 10000) && motor_lockout == true){
        motor_lockout=false;
        if (shutdown_called){
            shutdown();
        }
    }

}

void Hypnos::control(std::vector<float> u){
    if (_state.runState == static_cast<uint8_t>(ENGINE_RUN_STATE::RUNNING)){//if the engine is running
        if (millis() - _state.ignitionTime > _throttleLockout){
            //do control
            log("control input" + std::to_string(u[0]));
        }
    }
}

void Hypnos::updateSensors()
{
     if (millis() - _prevSensorUpdateTime > _sensorUpdateDelta)
    {
        if (millis() - _chamberPressure->getLastStateUpdateTime() < _networkTimeout){
            _chamberPressure->updateState();
            _chamberPressureTimeout = false;
            log("Chamber Pressure:" + std::to_string(_chamberPressure->getValue()));
        }else{
            if (!_chamberPressureTimeout){
                log("Chamber Pressure Timeout!");
                _chamberPressureTimeout = true;
            }
            // component timed out -> maybe timeout detection should be in the component rather than here
        }

        if (millis() - _tankPressure->getLastStateUpdateTime() < _networkTimeout){
            _tankPressure->updateState();
            _tankPressureTimeout = false;
            log("Tank Pressure:" + std::to_string(_tankPressure->getValue()));
        }else{
            // component timed out 
            if (!_tankPressureTimeout){
                log("Tank Pressure Timeout!");
                _tankPressureTimeout = true;
            }
        }
        _prevSensorUpdateTime = millis();
    } 

}


void Hypnos::heartbeat()
{
 
}

uint8_t Hypnos::flightCheck()
{
    uint8_t res = 0;

    res += _igniter->flightCheck(_networkRetryInterval,_componentStateExpiry,"Engine:" + std::to_string(getID()));
    res += _oxidiserValve->flightCheck(_networkRetryInterval,_componentStateExpiry,"Engine:" + std::to_string(getID()));
    res += _ventValve->flightCheck(_networkRetryInterval,_componentStateExpiry,"Engine:" + std::to_string(getID()));
    res += _chamberPressure->flightCheck(_networkRetryInterval,_componentStateExpiry,"Engine:" + std::to_string(getID()));
    res += _tankPressure->flightCheck(_networkRetryInterval,_componentStateExpiry,"Engine:" + std::to_string(getID()));
    
    return res;
}