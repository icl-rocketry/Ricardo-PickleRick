#pragma once

#include "engine.h"
#include <librrc/Helpers/jsonconfighelper.h>
#include <librrc/Interface/networkactuator.h>


struct ThanosState : public EngineState{};


class Thanos : public Engine
{
    public:

        Thanos(uint8_t id, JsonObjectConst engineConfig, addNetworkCallbackFunction_t addNetworkCallbackFunction, RnpNetworkManager &networkmanager, uint8_t handlerServiceID);

        void updateState() override;

        uint8_t flightCheck() override;

        void update() override;

        void armEngine() override;

   
        void disarmEngine() override ;

        void execute(int32_t func) override;

        /**
         * Defined as empty for now, will eventually send the demanded thrust/chamber pressure
        */
        void control(std::vector<float> u) override;

        ~Thanos(){};

    private:

        ThanosState _state;
        EngineState *getStatePtr() override { return &_state; };

        std::unique_ptr<NetworkActuator> _engine;

        std::unique_ptr<NetworkActuator> _igniter;

        std::unique_ptr<NetworkActuator> _oxVentValve;
        uint16_t _oxVentValveClosed = 0;
        uint16_t _oxVentValveOpen = 180;

        std::unique_ptr<NetworkActuator> _fuelVentValve;
        uint16_t _fuelVentValveClosed = 0;
        uint16_t _fuelVentValveOpen = 180;

        std::unique_ptr<NetworkActuator> _fuelPrssValve;
        uint16_t _fuelPrssValveClosed = 0;
        uint16_t _fuelPrssValveOpen = 180;

        void ignite() override;
        void shutdown() override;

        static constexpr uint16_t _networkRetryInterval = 5000;
        static constexpr uint16_t _componentStateExpiry = 1000;
        static constexpr uint16_t _networkTimeout = 2000;

        uint8_t ignition_command_arg = 1;
        uint8_t shutdown_command_arg = 2;

        uint64_t vent_delay = 1000;

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