#pragma once

#include <cmath>
#include <algorithm>

#include "controller.h"

class GriffinController : public Controller
{
public:
    GriffinController(uint8_t id, Controllable* griffin, uint32_t update_interval,
                      float targetLat, float targetLon,
                      float Kp, float Ki, float biasKnown,
                      float bankMax, float rateMax, float minGroundSpeed);

protected:
    void calculate(const SensorStructs::state_t& s) override;

    void reset() override;

private:
    const float _targetLat, _targetLon;
    const float _Kp, _Ki, _biasKnown;
    const float _bankMax, _rateMax;     // rad, rad/s
    const float _minGroundSpeed;        // m/s

    float _integ = 0;
    float _bank = 0;
};