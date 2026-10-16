#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <ArduinoJson.h>
#include <memory>
#include <functional>
#include <unordered_map>

#include <librnp/rnp_networkmanager.h>

#include <librrc/Handler/flightcomponenthandler.h>

#include <libriccore/riccorelogging.h>

#include "Config/types.h"
#include "Guided/guided.h"

using addNetworkCallbackFunction_t = std::function<void(uint8_t,uint8_t,std::function<void(std::unique_ptr<RnpPacketSerialized>)>,bool)>;

class GuidedHandler : public FlightComponentHandler<Guided,GuidedHandler>{
    public:
        GuidedHandler(RnpNetworkManager& networkmanager,const Types::LocalServoMap_t& localServoMap,  uint8_t serviceID):
        FlightComponentHandler(networkmanager,serviceID,[](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
        m_localServoMap(localServoMap)
        {};

        void update(); 

        void stopAllGuided();
        void startAllGuided();
        
    protected:
        friend class ConfigurableDynamicHandler;
        void setupIndividual_impl(size_t id,JsonObjectConst engineconfig);

        friend class FlightComponentHandler;
        uint8_t flightCheck_impl();

        void armComponents_impl();
        void disarmComponents_impl();


        /**
         * @brief Function to allow engines to add network callbacks directly to the network callback map.
         * Wraps the provided network callback to check that engine exists at the correct ID.
         * 
         * @return addNetworkCallbackFunction_t 
         */
        addNetworkCallbackFunction_t getaddNetworkCallbackFunction(uint8_t engineID);
    
    private:
        const Types::LocalServoMap_t& m_localServoMap;

};