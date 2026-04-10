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

namespace Types{
    
    using CoreTypes = RicCoreTypes<ForwardDecl_SystemClass,SYSTEM_FLAG,Commands::ID,256>;

};

