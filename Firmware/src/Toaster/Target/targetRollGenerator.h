#pragma once

#include <Arduino.h>

class TargetRollGenerator {
public:
    TargetRollGenerator();

    double getRollSetpoint(uint64_t timeMs);
};