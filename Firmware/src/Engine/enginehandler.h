#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <ArduinoJson.h>
#include <memory>
#include <functional>
#include <unordered_map>

#include <librnp/rnp_networkmanager.h>

#include "engine.h"
#include <librrc/Handler/flightcomponenthandler.h>

#include <libriccore/riccorelogging.h>


using addNetworkCallbackFunction_t = std::function<void(uint8_t,uint8_t,std::function<void(std::unique_ptr<RnpPacketSerialized>)>,bool)>;

class EngineHandler : public FlightComponentHandler<Engine,EngineHandler>{
    public:
        EngineHandler(RnpNetworkManager& networkmanager,uint8_t serviceID):
        FlightComponentHandler(networkmanager,serviceID,[](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);})
        {};

        void update(); // calls update on all engines

        void shutdownAllEngines();

    protected:
        friend class ConfigurableDynamicHandler;
        void setupIndividual_impl(size_t id,JsonObjectConst engineconfig);

        friend class FlightComponentHandler;
        /**
         * @brief performs flight check on all engines returns the number of engines in error
         * 
         * @return uint8_t 
         */
        uint8_t flightCheck_impl();

        void armComponents_impl();
        /**
         * @brief Function to allow engines to add network callbacks directly to the network callback map.
         * Wraps the provided network callback to check that engine exists at the correct ID.
         * 
         * @return addNetworkCallbackFunction_t 
         */
        addNetworkCallbackFunction_t getaddNetworkCallbackFunction(uint8_t engineID);

};