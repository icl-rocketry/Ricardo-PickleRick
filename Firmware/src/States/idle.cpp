#include "idle.h"

#include <memory>

#include <libriccore/fsm/state.h>
#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"

#include "system.h"


Idle::Idle(Types::CoreTypes::SystemStatus_t& systemtatus, Types::CoreTypes::CommandHandler_t& commandhandler):
State(SYSTEM_FLAG::STATE_IDLE,systemtatus),
_commandhandler(commandhandler)
{};

void Idle::initialize()
{
    Types::CoreTypes::State_t::initialize(); // call parent initialize first!
    _commandhandler.enableCommands({Commands::ID::Free_Ram});
};

Types::CoreTypes::State_ptr_t Idle::update()
{
    if (millis()-prevLogMessageTime > 1000)
    {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Idle heartbeat!");
        prevLogMessageTime = millis();
    }

    return nullptr;
};

void Idle::exit()
{
    Types::CoreTypes::State_t::exit(); // call parent exit last!
};