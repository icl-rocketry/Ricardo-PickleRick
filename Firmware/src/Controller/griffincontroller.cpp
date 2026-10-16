#include "griffincontroller.h"

GriffinController::GriffinController(uint8_t id, Controllable* griffin, uint32_t update_interval,
                    float targetLat, float targetLon,
                    float Kp, float Ki, float biasKnown,
                    float bankMax, float rateMax, float minGroundSpeed) :
    Controller(id, griffin, update_interval),
    _targetLat(targetLat), _targetLon(targetLon),
    _Kp(Kp), _Ki(Ki), _biasKnown(biasKnown),
    _bankMax(bankMax), _rateMax(rateMax), _minGroundSpeed(minGroundSpeed)
{};

void GriffinController::calculate(const SensorStructs::state_t& s)
{
    const float dt = static_cast<float>(deltaT) / 1000.f;   // deltaT is ms, set by Controller::update

    // target in the estimator's NED frame (origin = launch site), same conversion as the sim
    constexpr float mPerDeg = 111320.f;
    const float tn = (_targetLat - s.gps_launch_lat) * mPerDeg;
    const float te = (_targetLon - s.gps_launch_long) * mPerDeg * std::cos(s.gps_launch_lat * M_PI / 180.f);

    const float rn = tn - s.position(0);
    const float re = te - s.position(1);
    const float htarg = std::atan2(re, rn);

    // heading = ground track from velocity; undefined when nearly stationary, so hold last bank
    const float vn = s.velocity(0);
    const float ve = s.velocity(1);
    if (std::hypot(vn, ve) < _minGroundSpeed)
    {
        _controllable->control({_bank / _bankMax});
        return;
    }
    const float course = std::atan2(ve, vn);

    // wrap to [-pi, pi]
    const float err = std::remainder(htarg - course, 2.f * static_cast<float>(M_PI));

    // PI + known bias, integrate only while NOT saturated (anti-windup)
    float comBank = _Kp * err + _Ki * _integ + _biasKnown;
    if (std::abs(comBank) < _bankMax) { _integ += err * dt; }
    comBank = std::clamp(comBank, -_bankMax, _bankMax);

    // rate limit
    _bank += std::clamp(comBank - _bank, -_rateMax * dt, _rateMax * dt);

    // normalised to [-1, 1]; Griffin maps this onto the line differential
    _controllable->control({_bank / _bankMax});
};

void GriffinController::reset()
{
    _integ = 0;
    _bank = 0;
};