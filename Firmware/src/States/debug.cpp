#include "debug.h"

#include <libriccore/fsm/state.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include "Config/services_config.h"

#include "system.h"

Debug::Debug(System& system):
State(SYSTEM_FLAG::STATE_DEBUG,system.systemstatus),
_system(system)
{};

void Debug::initialize(){
    State::initialize();
    if (!_system.systemstatus.flagSetOr(SYSTEM_FLAG::DEBUG)){ //indicates first entry
        _system.networkmanager.registerService(static_cast<uint8_t>(Services::ID::HITL),_system.sensors.getHitlCallback()); //register hitl handler callback
    }
};

Types::CoreTypes::State_ptr_t Debug::update(){
    
    return nullptr;
};

void Debug::exit(){
    Types::CoreTypes::State_t::exit();
    if (!_system.systemstatus.flagSetOr(SYSTEM_FLAG::DEBUG)){ //indicates exiting out of debug mode completley
        _system.networkmanager.unregisterService(static_cast<uint8_t>(Services::ID::HITL)); //remove hitl service
    }
};