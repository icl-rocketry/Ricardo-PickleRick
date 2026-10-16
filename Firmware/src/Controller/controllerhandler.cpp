#include "Controller/controllerhandler.h"
#include "Controller/PID.h"
#include "Controller/griffincontroller.h"
#include <ArduinoJson.h>

#include <librrc/Helpers/jsonconfighelper.h>




void ControllerHandler::setupIndividual_impl(size_t id,JsonObjectConst controllerconfig) {
        using namespace LIBRRC::JsonConfigHelper;

        auto type = getIfContains<std::string>(controllerconfig,"type");

        if (type == "pid") {// configuration example DO NOT ACTUALLY USE THIS

        
            auto Kp = getIfContains<float>(controllerconfig, "Kp");
            auto Ki = getIfContains<float>(controllerconfig, "Ki");
            auto Kd = getIfContains<float>(controllerconfig, "Kd");
            auto setpoint = getIfContains<float>(controllerconfig, "setpoint");

            uint32_t update_interval = 0;
            setIfContains(controllerconfig, "updateInterval",update_interval,false);

            addObject(std::make_unique<PID>(id,
                                            Kp,
                                            Ki,
                                            Kd,
                                            setpoint,
                                            getControllable(controllerconfig),
                                            update_interval
                                            ));
        } else if (type == "griffin") {

            uint32_t update_interval = 0;
            setIfContains(controllerconfig, "updateInterval", update_interval, false);
            constexpr float d2r = M_PI / 180.f;
            addObject(std::make_unique<GriffinController>(id, getControllable(controllerconfig), update_interval,
                getIfContains<float>(controllerconfig, "targetLat"),
                getIfContains<float>(controllerconfig, "targetLon"),
                getIfContains<float>(controllerconfig, "Kp"),
                getIfContains<float>(controllerconfig, "Ki"),
                getIfContains<float>(controllerconfig, "biasKnown_deg") * d2r,
                getIfContains<float>(controllerconfig, "bankMax_deg") * d2r,
                getIfContains<float>(controllerconfig, "rateMax_degps") * d2r,
                getIfContains<float>(controllerconfig, "minGroundSpeed")));

        } else {
            throw std::runtime_error("Controller of type: " + type + " not implemented!"); 
        }

}

void ControllerHandler::update(const SensorStructs::state_t& estimator_state){
    for (auto& controller : *this){
        controller->update(estimator_state);
    }
}

Controllable* ControllerHandler::getControllable(JsonObjectConst config){
    using namespace LIBRRC::JsonConfigHelper;

    auto type = getIfContains<std::string>(config,"doohickeyType");
    auto id = getIfContains<uint8_t>(config,"doohickeyID");

    if (type == "Engine"){
        return dynamic_cast<Controllable*>(_enginehandler.getObject(id));
    }else if (type == "Guided"){
        return dynamic_cast<Controllable*>(_guidedhandler.getObject(id));
    }else{
        throw std::runtime_error("doohickey of type: " + type + " not implemented!"); 
    }
}