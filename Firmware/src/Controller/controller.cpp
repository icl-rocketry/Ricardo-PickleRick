#include "controller.h"
#include <Arduino.h>

Controller::~Controller(){};

void Controller::update(const SensorStructs::state_t& estimator_state){
    if (!_controllable->isActive()){
        _wasActive = false;
        return;
    }
    if (!_wasActive){ // just became active: start fresh
        _wasActive = true;
        reset();
        _last_update = millis();
    }

    deltaT = millis() - _last_update;
    if (deltaT > _update_interval){
        _last_update = millis();
        calculate(estimator_state);
    }
}