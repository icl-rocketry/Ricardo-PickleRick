#include "deploymenthandler.h"

#include <vector>
#include <memory>
#include <functional>

#include <librnp/rnp_networkmanager.h>
#include <ArduinoJson.h>

#include "Helpers/jsonconfighelper.h"

#include <librrc/rocketcomponent.h>
#include <librrc/rocketcomponenttype.h>

#include <librrc/networkactuator.h>
#include <librrc/packets/nrcpackets.h>

#include <librrc/i2cpyro.h>



void DeploymentHandler::setupIndividual_impl(size_t id,JsonObjectConst deployerconfig)
{
   using namespace JsonConfigHelper;
 
   auto type = getIfContains<std::string>(deployerconfig,"type");


    if (type == "i2c_act_servo"){
        throw std::runtime_error("i2c servo Not implemented!");
    }else if (type == "i2c_act_pyro"){
        auto address = getIfContains<uint8_t>(deployerconfig,"address");
        auto channel = getIfContains<uint8_t>(deployerconfig,"channel");
        auto invertContinuity = getIfContains<bool>(deployerconfig,"invertContinuity");
        addObject(std::make_unique<I2CPyro>(id, 
                                            address,
                                            channel,
                                            invertContinuity, 
                                            _wire,
                                            _logcb));
    }else if (type == "net_actuator"){
        auto address = getIfContains<uint8_t>(deployerconfig,"address");
        auto destination_service = getIfContains<uint8_t>(deployerconfig,"destination_service");
        addObject(std::make_unique<NetworkActuator>(id,  
                                                    address,
                                                    _serviceID,
                                                    destination_service, 
                                                    _networkmanager,
                                                    _logcb));
        //umm i tried okay
        addNetworkCallback(address,
                           destination_service,
                           [this,id](packetptr_t packetptr)
                                {
                                    dynamic_cast<NetworkActuator*>(getObject(id))->networkCallback(std::move(packetptr));
                                }
                            );
            
    }else{
        throw std::runtime_error("Invalid type!");
    }


    
};

uint8_t DeploymentHandler::flightCheck_impl()
{
    uint8_t components_in_error = 0;
    for (auto &component : *this)
    {
        components_in_error += component->flightCheck(_networkRetryInterval,_componentStateExpiry,"DeploymentHandler");
    }
    return components_in_error;
}

void DeploymentHandler::armComponents_impl()
{
    for (auto &component : *this)
    {
        component->arm();
    }
}



