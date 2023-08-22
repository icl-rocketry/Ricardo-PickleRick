#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

#include <Arduino.h>
#define ARDUINO_LOOP_STACK_SIZE 8192


#include <exception>

#include "system.h"



static constexpr bool exceptionsEnabled = true; //for debugging -> will integrate this into the sd configuration options later


TaskHandle_t loopTaskHandle = NULL;

System ricSystem;


/*
  Reading Position, Velocity and Time (PVT) via UBX binary commands
  By: Paul Clark
  SparkFun Electronics
  Date: December 21st, 2022
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to query a u-blox module for its position, velocity and time (PVT) data.
  We also turn off the NMEA output on the I2C port. This decreases the amount of I2C traffic dramatically.

  Note: Lat/Lon are large numbers because they are * 10^7. To convert lat/lon
  to something google maps understands simply divide the numbers by 10,000,000.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481
  SparkFun MAX-M10S Breakout (GPS-18037)     https://www.sparkfun.com/products/18037
  SparkFun ZED-F9K Breakout (GPS-18719)      https://www.sparkfun.com/products/18719
  SparkFun ZED-F9R Breakout (GPS-16344)      https://www.sparkfun.com/products/16344

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and your microcontroller board
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/


void setup_task()
{
    //MUST CALL CORE SYSTEM SETUP
    ricSystem.coreSystemSetup();
    // setup();
   
}

void inner_loop_task()
{
    //must call core system update
    ricSystem.coreSystemUpdate();
    // loop();
   
}

void loopTask(void *pvParameters)
{
    // esp_log_level_set("*", ESP_LOG_INFO); 
    setup_task();
    for(;;) {
        inner_loop_task();
        vTaskDelay(1); // this is important to allow the watchdog to be reset, and to let any other threads on the core work
    }
}

extern "C" void app_main()
{
    initArduino(); //probably dont even need this
    xTaskCreateUniversal(loopTask, "loopTask", ARDUINO_LOOP_STACK_SIZE, NULL, 1, &loopTaskHandle, 1);
}
