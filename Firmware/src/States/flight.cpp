#include "flight.h"

#include <memory>

#include <libriccore/fsm/state.h>
#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include "Config/commands_config.h"

#include "system.h"

#include "recovery.h"

#include "ApogeeDetection/apogeedetect.h"

Flight::Flight(System &system) : State(SYSTEM_FLAG::STATE_FLIGHT, system.systemstatus),
                                 _system(system)
                                 // apogeedetect(200,_system.logcontroller)
                                 {};

void Flight::initialize()
{
    State::initialize();
    _system.commandhandler.enableCommands({Commands::ID::Flight_Abort});
    
};

Types::CoreTypes::State_ptr_t Flight::update()
{

    _system.enginehandler.update();
    _system.controllerhandler.update(_system.estimator.getData());
    _system.eventhandler.update(_system.estimator.getData());

    float Ad = _system.estimator.getData().acceleration(2);

    if (Ad > 0 && !_system.systemstatus.flagSetOr(SYSTEM_FLAG::FLIGHTPHASE_BOOST))
    {
        _system.systemstatus.newFlag(SYSTEM_FLAG::FLIGHTPHASE_BOOST, "Entered Boost Phase");
        _system.systemstatus.deleteFlag(SYSTEM_FLAG::FLIGHTPHASE_COAST);
    }
    else if (Ad < 0 && !_system.systemstatus.flagSetOr(SYSTEM_FLAG::FLIGHTPHASE_COAST))
    {
        _system.systemstatus.newFlag(SYSTEM_FLAG::FLIGHTPHASE_COAST, "Entered Coast Phase");
        _system.systemstatus.deleteFlag(SYSTEM_FLAG::FLIGHTPHASE_BOOST);
    }
    ApogeeInfo apogeeinfo = _system.apogeedetect.checkApogee(-_system.estimator.getData().position(2), _system.estimator.getData().velocity(2), millis());
    if (apogeeinfo.reached)
    {
        _system.systemstatus.deleteFlag(SYSTEM_FLAG::FLIGHTPHASE_COAST);
        _system.systemstatus.deleteFlag(SYSTEM_FLAG::FLIGHTPHASE_BOOST);
        _system.systemstatus.newFlag(SYSTEM_FLAG::FLIGHTPHASE_APOGEE, "Apogee Detected!!");
        _system.estimator.setApogeeTime(apogeeinfo.time);
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Apogee at " + std::to_string(apogeeinfo.altitude));
      
        return std::make_unique<Recovery>(_system);
    }
    else
    {
        return nullptr;
    }
};

void Flight::exit()
{
    State::exit();
    _system.commandhandler.resetCommands();
};

// bool Flight::apogeeDetect(){ // 20hz

//     if (millis() - prevApogeeDetectTime >= apogeeDelta){
//         prevApogeeDetectTime = millis();
//         altitudeHistory.at(0) = altitudeHistory.at(1);
//         altitudeHistory.at(1) = altitudeHistory.at(2);
//         altitudeHistory.at(2) = _system.sensors.getData().baro.alt;

//         if ( (altitudeHistory.at(2) < altitudeHistory.at(1)) && (altitudeHistory.at(1) < altitudeHistory.at(0)) && abs(altitudeHistory.at(2) - altitudeHistory.at(0)) > 2){
//             return true;
//         }
//     }

//     return false;
// }