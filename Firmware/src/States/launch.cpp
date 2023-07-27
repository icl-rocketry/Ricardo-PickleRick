
#include "launch.h"
#include "flight.h"

#include "flags.h"
#include "stateMachine.h"

#include "Sound/Melodies/melodyLibrary.h"

Launch::Launch(stateMachine* sm):
State(sm,SYSTEM_FLAG::STATE_LAUNCH)
{};

void Launch::initialise(){
    State::initialise();
    //start telemetry logging here
    _sm->logcontroller.startLogging(LOG_TYPE::TELEMETRY);
    _sm->tunezhandler.play(MelodyLibrary::confirmation);

    //arm deployers and engines

    _sm->deploymenthandler.armComponents();
    _sm->enginehandler.armComponents();
 

};

State* Launch::update(){
    
    _sm->eventhandler.update(_sm->estimator.getData());

    //perform flight check
    int deployers_in_error = _sm->deploymenthandler.flightCheck();
    int engines_in_error = _sm->enginehandler.flightCheck();
    _sm->enginehandler.update();

    if ((deployers_in_error == 0) && (engines_in_error == 0) && _sm->systemstatus.flagSet(SYSTEM_FLAG::ERROR_FLIGHTCHECK)){
        _sm->systemstatus.deleteFlag(SYSTEM_FLAG::ERROR_FLIGHTCHECK);
    }else{
        if(!_sm->systemstatus.flagSet(SYSTEM_FLAG::ERROR_FLIGHTCHECK)){
            _sm->systemstatus.newFlag(SYSTEM_FLAG::ERROR_FLIGHTCHECK);
        }
    }

    // if (!_sm->systemstatus.flagSet(SYSTEM_FLAG::ERROR_FLIGHTCHECK) && _sm->estimator.getData().acceleration(2) < -1){ // launch acceleration threshold comparison of down acceleration with a threshold of 1.5 g idk if this is okay lol?
    if (_sm->estimator.getData().acceleration(2) < -9.81){ // launch acceleration threshold comparison of down acceleration with a threshold of 1.5 g idk if this is okay lol?
        _sm->estimator.setLiftoffTime(millis());
        State* flight_ptr = new Flight(_sm);
        return flight_ptr;
    }else{



        
        return this; //loopy loop
    }
};

void Launch::exitstate(){
    State::exitstate();


};

