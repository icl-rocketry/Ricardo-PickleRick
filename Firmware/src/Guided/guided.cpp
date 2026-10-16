#include "guided.h"

#include <ArduinoJson.h>
#include <memory>
#include <librnp/rnp_networkmanager.h>

#include <librrc/Interface/rocketcomponent.h>

#include <libriccore/riccorelogging.h>




void Guided::execute(int32_t func){
    switch(func){
        case static_cast<uint8_t>(GUIDED_EXECUTE::STOP):
        {
            stop();
            break;
        }
        case static_cast<uint8_t>(GUIDED_EXECUTE::START):
        {
            start();
            break;
        }
        case static_cast<uint8_t>(GUIDED_EXECUTE::SOFT_ABORT):
        {
            softAbort();
            break;
        }
        case static_cast<uint8_t>(GUIDED_EXECUTE::HARD_ABORT):
        {
            hardAbort();
            break;
        }
        case static_cast<uint8_t>(GUIDED_EXECUTE::RESET):
        {
            reset();
            break;
        }
        default:
        {
            break;
        }
    }
};

void Guided::stop(){
    if (getStatePtr()->runState == static_cast<uint8_t>(GUIDED_RUN_STATE::HARD_ABORT)){
        log("Stop ignored, in hard abort");
        return;
    }
    log("Stop Called!");
    getStatePtr()->runState = static_cast<uint8_t>(GUIDED_RUN_STATE::STOPPING);
    getStatePtr()->stopTime = millis();
};

void Guided::start(){
    if (getStatePtr()->runState != static_cast<uint8_t>(GUIDED_RUN_STATE::IDLE)){
        log("Start ignored, not idle");
        return;
    }
    log("Start Called!");
    getStatePtr()->runState = static_cast<uint8_t>(GUIDED_RUN_STATE::CONTROLLED);
    getStatePtr()->startTime = millis();
};

void Guided::softAbort(){
    if (getStatePtr()->runState != static_cast<uint8_t>(GUIDED_RUN_STATE::CONTROLLED)){
        log("Soft abort ignored, not under control");
        return;
    }
    log("Soft Abort Called!");
    getStatePtr()->runState = static_cast<uint8_t>(GUIDED_RUN_STATE::SOFT_ABORT);
    getStatePtr()->softAbortTime = millis();
};

void Guided::hardAbort(){
    log("Hard Abort Called!");
    getStatePtr()->runState = static_cast<uint8_t>(GUIDED_RUN_STATE::HARD_ABORT);
    getStatePtr()->hardAbortTime = millis();
};

void Guided::reset(){
    auto& rs = getStatePtr()->runState;
    if (rs != static_cast<uint8_t>(GUIDED_RUN_STATE::SOFT_ABORT) && rs != static_cast<uint8_t>(GUIDED_RUN_STATE::HARD_ABORT) && rs != static_cast<uint8_t>(GUIDED_RUN_STATE::ERROR)){
        log("Reset ignored");
        return;
    }
    log("Reset Called!");
    rs = static_cast<uint8_t>(GUIDED_RUN_STATE::IDLE);
};

Guided::~Guided(){};

void Guided::endHardAbort(){
    log("Time elapsed, ending hard abort");
    getStatePtr()->runState = static_cast<uint8_t>(GUIDED_RUN_STATE::IDLE); // dont try and change bank angle as no point, should be on ground by now
    getStatePtr()->stopTime = 0;
};