#pragma once
/**
 * @file hypnos.h
 * @author Kiran de Silva
 * @brief The HyPnOs engine implementation, this engine is a throttable hybird engine whihc is only netowkred
 *  so only networked components are used throughout.
 * @version 1
 * @date 2022-08-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "engine.h"

#include <ArduinoJson.h>
#include <memory>
#include <string>
#include <librrc/Helpers/jsonconfighelper.h>

#include <librrc/Interface/rocketactuator.h>
#include <librrc/Interface/networkactuator.h>
#include <librrc/Interface/networksensor.h>

#include <libriccore/riccorelogging.h>


struct HypnosState : public EngineState
{
};




class Hypnos : public Engine
{
public:
    Hypnos(uint8_t id, JsonObjectConst engineConfig, addNetworkCallbackFunction_t addNetworkCallbackFunction, RnpNetworkManager &networkmanager, uint8_t handlerServiceID);

    void updateState() override;

    uint8_t flightCheck() override;

    void update() override;

    void armEngine() override;

    void disarmEngine() override ;

    void execute(int32_t func) override;

    /**
     * @brief Control the engine throttle by varying valve displacement
     *
     * @param u single value
     */
    void control(std::vector<float> u) override;

private:
    HypnosState _state;
    EngineState *getStatePtr() override { return &_state; };

    // key components of hypnos
    std::unique_ptr<NetworkActuator> _igniter;
    // default firing time
    uint16_t _igniterFiringTime = 1000;
    bool _igniterFired;

    std::unique_ptr<NetworkActuator> _oxidiserValve;
    // default positions
    uint16_t _oxidiserValveClosed = 0;
    uint16_t _oxidiserValveOpen = 180;
    uint16_t _oxidiserValvePreIgnition = 55;

    std::unique_ptr<NetworkActuator> _ventValve;
    // default positions
    uint16_t _ventValveClosed = 0;
    uint16_t _ventValveOpen = 180;

    static constexpr uint16_t _sensorUpdateDelta = 250;
    uint32_t _prevSensorUpdateTime;
    std::unique_ptr<NetworkSensor> _chamberPressure;
    bool _chamberPressureTimeout;
    std::unique_ptr<NetworkSensor> _tankPressure;
    bool _tankPressureTimeout;

    // ignition parameters
    static constexpr uint16_t _preIgnitionDelay = 300;
    static constexpr uint16_t _postIgnitionDelay = 500;
    static constexpr uint16_t _throttleLockout = 4000;

    bool motor_lockout;
    bool shutdown_called;

    void ignite() override;
    void shutdown() override;
    void vent(); // fully vent main ox tank 

    static constexpr uint16_t _networkRetryInterval = 5000;
    static constexpr uint16_t _componentStateExpiry = 1000;
    static constexpr uint16_t _networkTimeout = 2000;

    /**
     * @brief Extension of the exectuor for Hypnos, check for conflicts with the base ENGINE_EXECTUE
     *
     */
    enum class HYPNOS_EXECTUE : uint8_t
    {
        VENT = 2
    };

    void updateSensors();
    static constexpr uint16_t _heartbeatDelta = 500;
    /**
     * @brief Checks all componetns for connectivity - NOT IMPLEMENTED YET!
     * 
     */
    void heartbeat();

 
    //helper function to add component to network callback
    template<typename T>
    void addComponentNetworkCallback(T* component,JsonObjectConst componentconf,addNetworkCallbackFunction_t addNetworkCallbackFunction)
    {
        using namespace LIBRRC::JsonConfigHelper;
        addNetworkCallbackFunction(getIfContains<uint8_t>(componentconf,"address"),
                            getIfContains<uint8_t>(componentconf,"destination_service"),
                            [this,component](packetptr_t packetptr)
                                {
                                    component->networkCallback(std::move(packetptr));
                                }
                            ,
                            true
                            );
    };
};
