#pragma once

#include <ArduinoJson.h>
#include <memory>

#include <librrc/Interface/rocketactuator.h>
#include "Config/types.h"

#include "engine.h"


class SimpleEngine:public Engine{
    public:
        SimpleEngine(uint8_t id, JsonObjectConst engineConfig, const Types::LocalPyroMap_t &localPyroMap, addNetworkCallbackFunction_t addNetworkCallbackF, RnpNetworkManager &networkmanager, uint8_t handlerServiceID);

        void updateState() override;

        uint8_t flightCheck() override;

        void armEngine() override;

        void disarmEngine() override;

        void update() override {}; //empty we dont need an update as this a simple on and off

        /**
         * @brief No control of a simple engine
         * 
         * @param u 
         */
        void control(std::vector<float> u) override{
            #ifdef _RICDEBUG
            std::cout<<"control input on engine " << std::to_string(getID()) << ": ";
            for (auto& elem : u){
                std::cout<<std::to_string(elem)<<",";
            }
            std::cout<<std::endl;
            #endif
        };

    private:
        EngineState _state;
        EngineState* getStatePtr() override {return &_state;};

        std::unique_ptr<RocketActuator> _igniter;
        int32_t _igniterParam;

        void ignite() override;
        void shutdown() override; 

        static constexpr uint16_t _networkRetryInterval = 5000;
        static constexpr uint16_t _componentStateExpiry = 1000;


};