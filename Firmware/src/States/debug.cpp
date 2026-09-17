#include "debug.h"

#include <libriccore/fsm/state.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include "Config/services_config.h"
#include "Config/commands_config.h"

#include "system.h"

Debug::Debug(System& system):
State(SYSTEM_FLAG::STATE_DEBUG,system.systemstatus),
_system(system)
{};

void Debug::initialize(){
    State::initialize();
    if (!_system.systemstatus.flagSetOr(SYSTEM_FLAG::DEBUG)){ //indicates first entry
        _system.systemstatus.newFlag(SYSTEM_FLAG::DEBUG);
        // enable persistent commands i.e commands are enabled through state changes now
        _system.commandhandler.enableCommands({Commands::ID::Reset_Orientation,
                                               Commands::ID::Reset_Localization,
                                               Commands::ID::Set_Beta,
                                               Commands::ID::Calibrate_AccelGyro_Bias,
                                               Commands::ID::Calibrate_Mag_Full,
                                               Commands::ID::Calibrate_Baro,
                                               Commands::ID::Enter_Debug,
                                               Commands::ID::Apogee_Override,
                                               Commands::ID::Exit_Debug},true);

        _system.networkmanager.registerService(static_cast<uint8_t>(Services::ID::HITL),_system.sensors.getHitlCallback()); //register hitl handler callback
    }
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered debug state.");
};

Types::CoreTypes::State_ptr_t Debug::update(){

    return nullptr;
};

void Debug::exit(){
    Types::CoreTypes::State_t::exit();

    if (!_system.systemstatus.flagSetOr(SYSTEM_FLAG::DEBUG)){ //indicates exiting out of debug mode completley
        _system.commandhandler.resetPersistentCommands();
        _system.networkmanager.unregisterService(static_cast<uint8_t>(Services::ID::HITL)); //remove hitl service

    }
    _system.commandhandler.resetCommands();
};