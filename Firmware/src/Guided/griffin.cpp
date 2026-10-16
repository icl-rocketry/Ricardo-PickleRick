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

void Griffin::start()
{
    Guided::start();
    if (_state.runState != static_cast<uint8_t>(GUIDED_RUN_STATE::STOPPING)){
        return;
    }
    _dTarget = 0.0f;
}

void Griffin::softAbort()
{
    Guided::softAbort();
    // get to 0 bank angle
}

void Griffin::hardAbort()
{
    Guided::hardAbort();
    // get to max bank angle
}

void Griffin::reset()
{
    Guided::reset();
    _dEstimate = 0.0f;
    _dTarget = 0.0f;
    _winchSpeed = 0.0f;
    _lastUpdateTime = millis();
}

void Griffin::debug()
{
    startDebug();
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
        case GUIDED_RUN_STATE::IDLE:
            // do nothing
            break;
        case GUIDED_RUN_STATE::ERROR:
            // do nothing
            break;
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
        case GUIDED_RUN_STATE::DEBUG:
        {
            const uint32_t t = currentTime - _debugStartTime;
            
            DEBUG_PHASE phase;
            if      (t < GuidedConfig::debug_fwd_ms)                                                                { phase = DEBUG_PHASE::FWD; }
            else if (t < GuidedConfig::debug_fwd_ms + GuidedConfig::debug_pause_ms)                                 { phase = DEBUG_PHASE::PAUSE; }
            else if (t < GuidedConfig::debug_fwd_ms + GuidedConfig::debug_pause_ms + GuidedConfig::debug_rev_ms)    { phase = DEBUG_PHASE::REV; }
            else                                                                                                    { phase = DEBUG_PHASE::DONE; }

            if (phase == _debugPhase) { break; }  // nothing changed
            _debugPhase = phase;

            switch (phase)
            {
                case DEBUG_PHASE::PAUSE: setSpeed(0.0f);          log("Debug wiggle: pause");   break;
                case DEBUG_PHASE::REV:   setSpeed(-GuidedConfig::debug_speed);  log("Debug wiggle: reverse"); break;
                case DEBUG_PHASE::DONE:
                    setSpeed(0.0f);
                    _state.runState = static_cast<uint8_t>(GUIDED_RUN_STATE::IDLE);
                    log("Debug wiggle complete, dEstimate=" + std::to_string(_dEstimate));
                    break;
                default: break;
            }
            break;

        }
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
    _winchSpeed = speed;
    _winch->execute(us);

}

void Griffin::startDebug()
{
    if (_state.runState != static_cast<uint8_t>(GUIDED_RUN_STATE::IDLE) || !winchArmed()) {
        log("Debug wiggle rejected");
        return;
    }
    _debugStartTime = millis();
    _debugPhase     = DEBUG_PHASE::FWD;
    _dEstimate      = 0.0f;
    setSpeed(GuidedConfig::debug_speed);
    _state.runState = static_cast<uint8_t>(GUIDED_RUN_STATE::DEBUG);
    log("Debug wiggle: forward");
}