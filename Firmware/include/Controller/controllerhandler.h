#pragma once

#include <vector>
#include <memory>
#include <ArduinoJson.h>

#include <libriccore/riccorelogging.h>
#include <librrc/Handler/configurabledynamichandler.h>
#include <librrc/Helpers/jsonconfighelper.h>

#include "Controller/controller.h"
#include "Controller/controllable.h"
#include "Controller/PID.h"
#include "Engine/enginehandler.h"

class ControllerHandler : public ConfigurableDynamicHandler<Controller,ControllerHandler>{
    public:
        ControllerHandler(EngineHandler& enginehandler):
            ConfigurableDynamicHandler([](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
            _enginehandler(enginehandler)
        {};

        void update(const SensorStructs::state_t& estimator_state);

    protected:

        EngineHandler& _enginehandler; // currently only supporting engines but can be expanded later to support controllable fins or more!

        Controllable* getControllable(JsonObjectConst config);

        friend class ConfigurableDynamicHandler;
        void setupIndividual_impl(size_t id,JsonObjectConst controllerconfig);
        
    
};