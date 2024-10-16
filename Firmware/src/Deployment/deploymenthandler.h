#pragma once

#include <vector>
#include <memory>
#include <functional>
#include <unordered_map>

#include <librnp/rnp_networkmanager.h>
#include <ArduinoJson.h>
#include <Wire.h>


#include <librrc/rocketactuator.h>

#include <librrc/flightcomponenthandler.h>
#include <librrc/configurabledynamichandler.h>
#include <librrc/networkeddynamichandler.h>

#include <libriccore/riccorelogging.h>





class DeploymentHandler : public FlightComponentHandler<RocketActuator,DeploymentHandler> {
    public:
        /**
         * @brief Construct a new Deployment Handler object
         * 
         * @param networkmanager 
         * @param serviceID network service the handler is assigned to
         * @param logcontroller 
         */
        DeploymentHandler(RnpNetworkManager& networkmanager,uint8_t serviceID,TwoWire& wire):
            FlightComponentHandler(networkmanager,serviceID,[](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
            _wire(wire)
        {};


    protected:
        friend class ConfigurableDynamicHandler; //base class needs access to implementation so make it friend
        void setupIndividual_impl(size_t id,JsonObjectConst deployerconfig);

        friend class FlightComponentHandler;
        /**
         * @brief Performs the flight check on componets, checks if componets are connected and are functioning nominally
         * 
         * @return uint8_t returns 0 if all good, if value is greater than zero, it represents the number of components not ready or not updated
         */
        uint8_t flightCheck_impl();
        void armComponents_impl();
         
    private:
        static constexpr uint16_t _networkRetryInterval = 5000; // 5 seconds before a new update state request is sent
        static constexpr uint16_t _componentStateExpiry = 1000; //1 second expiry
        TwoWire& _wire;
};
