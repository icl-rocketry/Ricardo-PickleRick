/*
**********************
* PINS               *
**********************
 */
#pragma once
#include "v1_pinmap.h"
#include "v2_pinmap.h"
#include "v3_pinmap.h"

#ifndef HARDWARE_VERSION
    #error "Invalid or no hardware version specified"
#endif

#if HARDWARE_VERSION == 1
    namespace PinMap = PickleRickV1Pins;
#elif HARDWARE_VERSION == 2
    namespace PinMap = PickleRickV2Pins;
#elif HARDWARE_VERSION == 3
    namespace PinMap = PickleRickV3Pins;
#else
    #error "Invalid hardware version specified"
#endif



