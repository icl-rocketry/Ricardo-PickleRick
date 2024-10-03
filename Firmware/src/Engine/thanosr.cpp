#include "thanosr.h"
#include <librrc/Helpers/jsonconfighelper.h>

ThanosR::ThanosR(uint8_t id, JsonObjectConst engineConfig, addNetworkCallbackFunction_t addNetworkCallbackFunction, RnpNetworkManager &networkmanager, uint8_t handlerServiceID) : 
Engine(id, networkmanager, handlerServiceID)
{
    using namespace LIBRRC::JsonConfigHelper;

    auto engineConf = getIfContains<JsonObjectConst>(engineConfig, "engine");
    m_engine = std::make_unique<NetworkActuator>(0,
                                                 getIfContains<uint8_t>(engineConf, "address"),
                                                 handlerServiceID,
                                                 getIfContains<uint8_t>(engineConf, "destination_service"),
                                                 _networkmanager,
                                                 getLogCB());
    
    addComponentNetworkCallback(m_engine.get(),engineConf,addNetworkCallbackFunction);

    auto oxServoVentValveConf = getIfContains<JsonObjectConst>(engineConfig, "oxServoVentValve");
    m_oxServoVentValve = std::make_unique<NetworkActuator>(1,
                                                getIfContains<uint8_t>(oxServoVentValveConf, "address"),
                                                handlerServiceID,
                                                getIfContains<uint8_t>(oxServoVentValveConf, "destination_service"),
                                                _networkmanager,
                                                getLogCB());

    setIfContains<uint16_t>(oxServoVentValveConf,"closed_position",m_oxServoVentValveClosed,false);
    setIfContains<uint16_t>(oxServoVentValveConf,"open_position",m_oxServoVentValveOpen,false);

    addComponentNetworkCallback(m_oxServoVentValve.get(),oxServoVentValveConf,addNetworkCallbackFunction);

    auto oxSolenoidVentValveConf = getIfContains<JsonObjectConst>(engineConfig, "oxSolenoidVentValve");
    m_oxSolenoidVentValve = std::make_unique<NetworkActuator>(2,
                                                getIfContains<uint8_t>(oxSolenoidVentValveConf, "address"),
                                                handlerServiceID,
                                                getIfContains<uint8_t>(oxSolenoidVentValveConf, "destination_service"),
                                                _networkmanager,
                                                getLogCB());

    addComponentNetworkCallback(m_oxSolenoidVentValve.get(),oxSolenoidVentValveConf,addNetworkCallbackFunction);


    auto fuelSolenoidVentValveConf = getIfContains<JsonObjectConst>(engineConfig, "fuelSolenoidVentValve");
    m_fuelSolenoidVentValve = std::make_unique<NetworkActuator>(3,
                                                getIfContains<uint8_t>(fuelSolenoidVentValveConf, "address"),
                                                handlerServiceID,
                                                getIfContains<uint8_t>(fuelSolenoidVentValveConf, "destination_service"),
                                                _networkmanager,
                                                getLogCB());

    addComponentNetworkCallback(m_fuelSolenoidVentValve.get(),fuelSolenoidVentValveConf,addNetworkCallbackFunction);

    auto prssSolenoidVentValveConf = getIfContains<JsonObjectConst>(engineConfig, "prssSolenoidVentValve");
    m_prssSolenoidVentValve = std::make_unique<NetworkActuator>(4,
                                                getIfContains<uint8_t>(prssSolenoidVentValveConf, "address"),
                                                handlerServiceID,
                                                getIfContains<uint8_t>(prssSolenoidVentValveConf, "destination_service"),
                                                _networkmanager,
                                                getLogCB());

    addComponentNetworkCallback(m_prssSolenoidVentValve.get(),prssSolenoidVentValveConf,addNetworkCallbackFunction);

    auto eRegConf = getIfContains<JsonObjectConst>(engineConfig, "eReg");
    m_eReg = std::make_unique<NetworkActuator>(5,
                                                getIfContains<uint8_t>(eRegConf, "address"),
                                                handlerServiceID,
                                                getIfContains<uint8_t>(eRegConf, "destination_service"),
                                                _networkmanager,
                                                getLogCB());

    addComponentNetworkCallback(m_eReg.get(),eRegConf,addNetworkCallbackFunction);

    log("Thanos-R Constructed");
}

void ThanosR::updateState()
{
    m_engine->updateState();
    m_oxServoVentValve->updateState();
    m_oxSolenoidVentValve->updateState();
    m_fuelSolenoidVentValve->updateState();
    m_prssSolenoidVentValve->updateState();
    m_eReg->updateState();
}

void ThanosR::execute(int32_t func)
{
    Engine::execute(func);
}

void ThanosR::armEngine()
{
    m_engine->arm();
    m_oxServoVentValve->arm();
    m_oxSolenoidVentValve->arm();
    m_fuelSolenoidVentValve->arm();
    m_prssSolenoidVentValve->arm();
    m_eReg->arm();
}

void ThanosR::disarmEngine()
{
    m_engine->disarm();
    m_oxServoVentValve->disarm();
    m_oxSolenoidVentValve->disarm();
    m_fuelSolenoidVentValve->disarm();
    m_prssSolenoidVentValve->disarm();
    m_eReg->disarm();
    //reset variables
    m_oxVented = false;
    m_fuelVented = false;
}

void ThanosR::shutdown()
{
    m_engine->execute(m_shutdown_command_arg);
    m_eReg->execute(2); // shut down ereg
    Engine::shutdown();
    m_state.runState = static_cast<uint8_t>(ENGINE_RUN_STATE::SHUTDOWN);
}

void ThanosR::ignite()
{
    m_engine->execute(m_ignition_command_arg);
    Engine::ignite();
    m_state.runState = static_cast<uint8_t>(ENGINE_RUN_STATE::IGNITION);
}

void ThanosR::update()
{
    if (m_state.runState == static_cast<uint8_t>(ENGINE_RUN_STATE::IGNITION)){
        if (millis() - getStatePtr()->ignitionTime > m_ignitionDelay){
            //igntion delay is the delay between firing the pyro and
            //openign the main valves. after we open the main valves we want to 
            // enable the e-reg 
            m_eReg->execute(1); // enter into controlled state
            m_state.runState = static_cast<uint8_t>(ENGINE_RUN_STATE::RUNNING);
        }
    }
    else if(m_state.runState == static_cast<uint8_t>(ENGINE_RUN_STATE::SHUTDOWN)){
        /*when shutdown is called, the engine will close the main oxidiser vlave
        but keep the main fuel valve open so that we fully drain the fuel tank.
        We wait for m_oxVentDelay before opening the oxidiser vent valves to vent
        the oxidier tank, and we also open the pressurant solenoid valve to vent the nitrogen
        tank. Finally once the time since shutdown is greater that m_fuelVentDelay we open
        the fuel solenoid vent valve to vent the fuel tank.
        */
        if(millis() - getStatePtr()->shutdownTime > m_oxVentDelay && !m_oxVented){
            m_oxServoVentValve->execute(m_oxServoVentValveOpen);
            m_oxSolenoidVentValve->execute(m_solenoidOpenArg);
            m_prssSolenoidVentValve->execute(m_solenoidOpenArg);
            m_oxVented = true;
        } 
        if (millis() - getStatePtr()->shutdownTime > m_fuelVentDelay && !m_fuelVented){
            m_fuelSolenoidVentValve->execute(m_solenoidOpenArg);
            m_fuelVented = true;
        }
    }
}

uint8_t ThanosR::flightCheck()
{
    uint8_t res = 0;

    res += m_engine->flightCheck(m_networkRetryInterval, m_componentStateExpiry, "Engine " + std::to_string(getID()) + ": Stark Engine Controller");
    res += m_oxServoVentValve->flightCheck(m_networkRetryInterval, m_componentStateExpiry, "Engine " + std::to_string(getID()) + ": Ox Vent");
    res += m_oxSolenoidVentValve->flightCheck(m_networkRetryInterval, m_componentStateExpiry, "Engine " + std::to_string(getID()) + ": Ox Solenoid");
    res += m_fuelSolenoidVentValve->flightCheck(m_networkRetryInterval, m_componentStateExpiry, "Engine " + std::to_string(getID()) + ": Fuel Solenoid");
    res += m_prssSolenoidVentValve->flightCheck(m_networkRetryInterval, m_componentStateExpiry, "Engine " + std::to_string(getID()) + ": Pressurant Solenoid");
    res += m_eReg->flightCheck(m_networkRetryInterval, m_componentStateExpiry, "Engine " + std::to_string(getID()) + ": E-Reg");
    
    return res;
    
}

void ThanosR::control(std::vector<float> u){};