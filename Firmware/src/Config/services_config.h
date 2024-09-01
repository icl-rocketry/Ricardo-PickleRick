/**
 * @file services_config.h
 * @author Kiran de Silva (kd619@ic.ac.uk)
 * @brief Defintion of user defined services
 * @version 0.1
 * @date 2023-06-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <stdint.h>


namespace Services{
    /**
     * @brief ID of user defined services NB start at 2, all ID's below 2 are reserved for default services found in 'rnp_networkmanager.h'
     * 
     */
    enum class ID:uint8_t{
        HITL = 3,
        DeploymentHandler = 4,
        EngineHandler = 5,
        Pyro0 = 10,
        Pyro1 = 11,
        Pyro2 = 12,
        Pyro3 = 13,
        Servo0 = 20,
        Servo1 = 21,
        Servo2 = 22,
        Servo3 = 23
    };

};