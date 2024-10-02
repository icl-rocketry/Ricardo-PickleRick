#pragma once

#include "engine.h"
#include <librrc/Helpers/jsonconfighelper.h>
#include <librrc/Interface/networkactuator.h>


struct ThanosRState : public EngineState{};


class ThanosR : public Engine
{
    public:

        ThanosR(uint8_t id, JsonObjectConst engineConfig, addNetworkCallbackFunction_t addNetworkCallbackFunction, RnpNetworkManager &networkmanager, uint8_t handlerServiceID);

        void updateState() override;

        uint8_t flightCheck() override;

        void update() override;

        void armEngine() override;

        void disarmEngine() override;

        void execute(int32_t func) override;

        /**
         * Defined as empty for now, will eventually send the demanded thrust/chamber pressure
        */
        void control(std::vector<float> u) override;

        ~ThanosR(){};

    private:

        ThanosRState m_state;
        EngineState *getStatePtr() override { return &m_state; };

        std::unique_ptr<NetworkActuator> m_engine;

        std::unique_ptr<NetworkActuator> m_oxServoVentValve;
        uint16_t m_oxServoVentValveClosed = 0;
        uint16_t m_oxServoVentValveOpen = 180;

        std::unique_ptr<NetworkActuator> m_oxSolenoidVentValve;
        std::unique_ptr<NetworkActuator> m_fuelSolenoidVentValve;
        std::unique_ptr<NetworkActuator> m_prssSolenoidVentValve;
        static constexpr int32_t m_solenoidOpenArg = 1;     /* argument for normally open solenoids */

        std::unique_ptr<NetworkActuator> m_eReg;

        void ignite() override;
        void shutdown() override;

        static constexpr uint16_t m_networkRetryInterval = 5000;
        static constexpr uint16_t m_componentStateExpiry = 1000;
        static constexpr uint16_t m_networkTimeout = 2000;

        static constexpr uint8_t m_ignition_command_arg = 1;
        static constexpr uint8_t m_shutdown_command_arg = 2;

        static constexpr uint64_t m_ignitionDelay = 500;
        static constexpr uint64_t m_oxVentDelay = 1000;
        bool m_oxVented = false;
        static constexpr uint64_t m_fuelVentDelay = 5000;
        bool m_fuelVented = false;

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