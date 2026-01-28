#include "engine.h"

#include <ArduinoJson.h>
#include <memory>
#include <librnp/rnp_networkmanager.h>

#include <librrc/Interface/rocketcomponent.h>

#include <libriccore/riccorelogging.h>




void Engine::execute(int32_t func){
    switch(func){
        case static_cast<uint8_t>(ENGINE_EXECUTE::IGNITE):
        {
            ignite();
            break;
        }
        case static_cast<uint8_t>(ENGINE_EXECUTE::SHUTDOWN):
        {
            shutdown();
            break;
        }
        default:
        {
            break;
        }
    }
};

void Engine::ignite(){
    log("Ignition Called!");
    getStatePtr()->ignitionTime = millis();

};

void Engine::shutdown(){
    log("Shutdown Called!");
    getStatePtr()->shutdownTime = millis();
};


Engine::~Engine(){};