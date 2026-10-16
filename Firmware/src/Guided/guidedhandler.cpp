#include "guidedhandler.h"

#include <vector>
#include <unordered_map>
#include <string>
#include <ArduinoJson.h>
#include <memory>
#include <functional>

#include <librnp/rnp_networkmanager.h>

#include <librrc/Helpers/jsonconfighelper.h>

#include "Guided/griffin.h"

GuidedHandler::GuidedHandler(RnpNetworkManager& networkmanager,const Types::LocalServoMap_t& localServoMap,  uint8_t serviceID):
        FlightComponentHandler(networkmanager,serviceID,[](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
        m_localServoMap(localServoMap)
        {};

void GuidedHandler::update(){ // call update on all guideds
    for (auto& guided : *this){
        guided->update();
    }
}

uint8_t GuidedHandler::flightCheck_impl(){
    
    uint8_t guideds_in_error = 0;
    for (auto& guided: *this){
        guideds_in_error += guided->flightCheckGuided();
    }
    return guideds_in_error;
}

void GuidedHandler::armComponents_impl(){
    for (auto& guided: *this){
        guided->armGuided();
    }
}

void GuidedHandler::disarmComponents_impl(){
    for (auto& guided: *this){
        guided->disarmGuided();
    }
}

void GuidedHandler::stopAllGuided(){
    for (auto& guided: *this){
        guided->execute(0);
    }
}

void GuidedHandler::startAllGuided(){
    for (auto& guided: *this){
        guided->execute(1);
    }
}

void GuidedHandler::setupIndividual_impl(size_t id, JsonObjectConst guidedconfig){
    using namespace LIBRRC::JsonConfigHelper;

    auto type = getIfContains<std::string>(guidedconfig,"type");

    if (type == "Griffin"){
        addObject(std::make_unique<Griffin>(id,
                                            guidedconfig,
                                            m_localServoMap,
                                            _networkmanager,
                                            _serviceID));
    }
    else{
        throw std::runtime_error("Engine of type:" + type + "not implemented!");
    }


}

addNetworkCallbackFunction_t GuidedHandler::getaddNetworkCallbackFunction(uint8_t guidedID)
{
    return [this,guidedID](uint8_t source, uint8_t source_service, std::function<void(std::unique_ptr<RnpPacketSerialized>)> callback, bool throwOnError)
    {
        auto wrappedCallback = [this,callback,guidedID](std::unique_ptr<RnpPacketSerialized> packet_ptr) {
            // check that guided exists
            //calling getObj will check that the id's match and that object exists.
            //If obj doesnt exists, a runtime exception will be thrown
            getObject(guidedID);
            callback(std::move(packet_ptr));
        };
        addNetworkCallback(source, source_service, wrappedCallback, throwOnError);
    };
};