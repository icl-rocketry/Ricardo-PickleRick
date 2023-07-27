#include "Arduino.h"
#include "recovery.h"

#include "flags.h"
#include "stateMachine.h"

#include "Sound/Melodies/melodyLibrary.h"


Recovery::Recovery(stateMachine* sm):
State(sm,SYSTEM_FLAG::STATE_RECOVERY)
{};

void Recovery::initialise(){
    State::initialise();
    _sm->tunezhandler.play(MelodyLibrary::zeldatheme,true); // play startup sound
    _sm->enginehandler.shutdownAllEngines();
};


State* Recovery::update(){

    _sm->enginehandler.update();
    _sm->controllerhandler.update(_sm->estimator.getData());
    _sm->eventhandler.update(_sm->estimator.getData());
    
    return this;
};

void Recovery::exitstate(){
    State::exitstate();
    _sm->tunezhandler.clear(); // stop looping zelda
};