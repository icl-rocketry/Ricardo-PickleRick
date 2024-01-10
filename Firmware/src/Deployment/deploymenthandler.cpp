#include "deploymenthandler.h"

#include <vector>
#include <memory>
#include <functional>

#include <librnp/rnp_networkmanager.h>
#include <ArduinoJson.h>

#include "Helpers/jsonconfighelper.h"

#include <librrc/Interface/rocketcomponent.h>
#include <librrc/Interface/rocketcomponenttype.h>

#include <librrc/Interface/networkactuator.h>
#include <librrc/Packets/nrcpackets.h>
#include <librrc/Local/remoteactuatoradapter.h>
#include <librrc/Remote/nrcremotepyro.h>

#include "Config/types.h"
// #include <librrc/Local/i2cpyro.h>



void DeploymentHandler::setupIndividual_impl(size_t id,JsonObjectConst deployerconfig)
{
   using namespace JsonConfigHelper;
 
   auto type = getIfContains<std::string>(deployerconfig,"type");


    if (type == "local_pyro"){
        uint8_t channel = getIfContains<uint8_t>(deployerconfig,"channel");
        if (channel > 3)
        {
            throw std::runtime_error("Local pyro channel out of range!");
        }
        //retrive nrcremotepyro instance correspondign to channel number
        Types::LocalPyro_t& localPyro = *(m_localPyroMap.at(channel));


        //add object to dep handler and use adapter to convert to local type
        addObject(std::make_unique<RemoteActuatorAdapter<Types::LocalPyro_t>>(id,localPyro,_logcb));


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



