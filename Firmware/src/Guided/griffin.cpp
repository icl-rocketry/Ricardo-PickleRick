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

    const float lineRate = (_winchSpeed >= 0) ? GuidedConfig::winch_line_rate_fwd_mps
                                              : GuidedConfig::winch_line_rate_rev_mps;
    _dEstimate += _winchSpeed * lineRate * dt;

    const float err = _dTarget - _dEstimate;
    const bool atTarget = std::abs(err) < GuidedConfig::guided_min_speed;

    switch (static_cast<GUIDED_RUN_STATE>(_state.runState))
    {
        case GUIDED_RUN_STATE::STOPPING:
            if (_dEstimate < 0.1f && _dEstimate > -0.1f)
            {
                _state.runState = static_cast<uint8_t>(GUIDED_RUN_STATE::IDLE);
                log("Griffin stopped");
            }
            break;
        case GUIDED_RUN_STATE::CONTROLLED:
            // normal operation, do nothing
            break;
        case GUIDED_RUN_STATE::SOFT_ABORT:
            // soft abort, stop winch
            _winch->execute(0);
            break;
        case GUIDED_RUN_STATE::HARD_ABORT:
            if (currentTime - _state.hardAbortTime > GuidedConfig::hard_abort_hold_ms)
            {
                endHardAbort();   // → STOPPING, target untouched so it halts where it is
            }
            break;
        default:
            break;
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

    const bool valid = !u.empty() && std::isfinite(u[0]);
    if (!valid)
    {
        return;
    }

    _dTarget = std::clamp(u[0], -1.f, 1.f) * GuidedConfig::max_differential; // maybe??

};

void Griffin::setSpeed(float speed)
{
    if (!winchArmed()) { _winchSpeed = 0; return; }        // command would be dropped

    int32_t us;
    if (speed > 0)      { us = GuidedConfig::winch_fwd_edge_us + std::lround(speed * (GuidedConfig::winch_fwd_max_us - GuidedConfig::winch_fwd_edge_us)); }  // +400
    else if (speed < 0) { us = GuidedConfig::winch_rev_edge_us + std::lround(speed * (GuidedConfig::winch_rev_edge_us - GuidedConfig::winch_rev_max_us)); }  // −500
    else                { us = GuidedConfig::winch_stop_us; }
    _winchSpeed = us;
    _winch->execute(us);

}