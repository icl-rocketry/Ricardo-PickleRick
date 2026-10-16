#include "griffin.h"

Griffin::Griffin(uint8_t id, 
                 JsonObjectConst guidedConfig, 
                 const Types::LocalServoMap_t& localServoMap,
                 RnpNetworkManager &networkmanager, 
                 uint8_t handlerServiceID) : 
    Guided(id, networkmanager, handlerServiceID)
    {
        using namespace LIBRRC::JsonConfigHelper;

        auto channel = getIfContains<uint8_t>(guidedConfig, "servo_channel");
        if (channel >= localServoMap.size()){
            throw std::runtime_error("Griffin servo channel " + std::to_string(channel) + " is out of bounds for localServoMap of size " + std::to_string(localServoMap.size()));
        }
        _winch = std::make_unique<RemoteActuatorAdapter<Types::LocalServo_t>>(0, *localServoMap.at(channel), getLogCB());
        auto guidedConf = getIfContains<JsonObjectConst>(guidedConfig, "guided");

        // add winch
        log("Griffin Constructed");
    }

void Griffin::updateState()
{
    _winch->updateState();
}

void Griffin::execute(int32_t func)
{
    Guided::execute(func);//add vent here
}

void Griffin::armGuided()
{
    _winch->arm();
}

void Griffin::disarmGuided()
{
    _winch->disarm();
}

void Griffin::stop()
{
    Guided::stop();
    if (_state.runState != static_cast<uint8_t>(GUIDED_RUN_STATE::STOPPING)){
        return;
    }
    _dTarget = 0.0f;
}



void Griffin::update()
{
    uint32_t currentTime = millis();
    float dt = (currentTime - _lastUpdateTime) / 1000.0f; // convert to seconds
    _lastUpdateTime = currentTime;

    _dEstimate += _lineRate * _winchSpeed * dt; // integrate winch speed to estimate difference between parafoil lengths

    float err = _dTarget - _dEstimate;
    _speedCmd = err * _kP; 
    setSpeed(_speedCmd);

    if (_state.runState == static_cast<uint8_t>(GUIDED_RUN_STATE::STOPPING)){
        if (_dEstimate < 0.1f && _dEstimate > -0.1f){ // if difference between parafoil lengths is less than 10cm, consider it stopped
            log("Griffin stopped");
            _state.runState = static_cast<uint8_t>(GUIDED_RUN_STATE::IDLE);
        }
    } else if (_state.runState == static_cast<uint8_t>(GUIDED_RUN_STATE::CONTROLLED)){
        // normal operation, do nothing
    } else if (_state.runState == static_cast<uint8_t>(GUIDED_RUN_STATE::SOFT_ABORT)){
        // soft abort, stop winch
        _winch->execute(0);
    } else if (_state.runState == static_cast<uint8_t>(GUIDED_RUN_STATE::HARD_ABORT)){
        // hard abort, stop winch and vent
        _winch->execute(0);
    }

}

uint8_t Griffin::flightCheck()
{
    uint8_t res = 0;

    res += _winch->flightCheck(1000, 5000, "Griffin Winch");
    
    return res;
    
}

void Griffin::control(std::vector<float> u)
{
    if (!isActive()) { return; }
    if (u.empty() || u.size() < 1) { return; }
};

void Griffin::setSpeed(float speed)
{
    if (!winchArmed()) { _winchSpeed = 0; return; }        // command would be dropped

    speed = std::clamp(speed, -1.f, 1.f);
    if (std::abs(speed) < GuidedConfig::guided_min_speed) { _winchSpeed = 0; } // deadband to avoid jitter

    _winchSpeed = speed;
    _winch->execute(GuidedConfig::guided_zero_speed_angle + static_cast<int32_t>(speed * 1000)); // scale to int32_t for network transmission
}