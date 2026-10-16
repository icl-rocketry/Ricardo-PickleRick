#pragma once

#include <librrc/componentstatusflags.h>
#include <librrc/Helpers/jsonconfighelper.h>
#include <librrc/Interface/networkactuator.h>

#include "guided.h"
#include "Config/types.h"
#include "Config/guided_config.h"

struct GriffinState : public GuidedState{};


class Griffin : public Guided
{
    public:

        Griffin(uint8_t id, JsonObjectConst griffinConfig, const Types::LocalServoMap_t& localServoMap, RnpNetworkManager &networkmanager, uint8_t handlerServiceID);

        void updateState() override;

        uint8_t flightCheck() override;

        void update() override;

        void armGuided() override;

        void disarmGuided() override ;

        void execute(int32_t func) override;

        /**
         * Defined as empty for now, will eventually send the demanded thrust/chamber pressure
        */
        void control(std::vector<float> u) override;

        bool isActive() const override { return _state.runState == static_cast<uint8_t>(GUIDED_RUN_STATE::CONTROLLED); };
        ~Griffin(){};

    protected:

        void stop() override;
        void start() override;
        void softAbort() override;
        void hardAbort() override;
        void reset() override;

    private:

        GriffinState _state;
        GuidedState *getStatePtr() override { return &_state; };

        std::unique_ptr<RocketActuator> _winch;

        bool winchArmed() { return !_winch->getState().flagSet(LIBRRC::COMPONENT_STATUS_FLAGS::DISARMED); };

        float _dTarget = 0.0f; // target difference between parafoil lengths
        float _dEstimate = 0.0f; 
        float _winchSpeed = 0.0f; // last commanded speed to winch, used to estimate difference between parafoil lengths
        float _lineRate;
        uint32_t _lastUpdateTime = 0;



        void setSpeed(float speed);

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