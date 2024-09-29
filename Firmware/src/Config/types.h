/**
 * @file types.h
 * @author Kiran de Silva (kd619@ic.ac.uk)
 * @brief type declarations for more convient types
 * @version 0.1
 * @date 2023-06-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#pragma once

#include <array>

#include <libriccore/riccoretypes.h>

//we need the forward declarations for the system aswell
#include "forward_decl.h"

#include "systemflags_config.h"
#include "commands_config.h"

#include <librrc/Remote/nrcremotepyro.h>
#include <librrc/Remote/nrcremoteservo.h>
#include "Deployment/PCA9534Gpio.h"
#include "Deployment/PCA9685PWM.h"

namespace Types{
    using CoreTypes = RicCoreTypes<ForwardDecl_SystemClass,SYSTEM_FLAG,Commands::ID,256>;
    //any other useful aliases used in multiple places should be defined here
    using LocalPyro_t = NRCRemotePyro<PCA9534Gpio>;
    using LocalPyroMap_t = std::array<LocalPyro_t*,4>;
    
    using LocalServo_t = NRCRemoteServo<PCA9685PWM>;
    using LocalServoMap_t = std::array<LocalServo_t*,4>;

};

