#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

// #include <sprofiler.h>

#include <Arduino.h>
#define ARDUINO_LOOP_STACK_SIZE 32768


#include <exception>

#include "system.h"



static constexpr bool exceptionsEnabled = true; //for debugging -> will integrate this into the sd configuration options later
static constexpr bool highRateLoopMode = false;


TaskHandle_t loopTaskHandle = NULL;

System ricSystem;

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
    // sprofiler_initialize(100);
    setup_task();
    uint32_t last_yield_time = millis();
    for(;;) {
        inner_loop_task();
        if (highRateLoopMode)
        {
            const uint32_t now = millis();
            if (now - last_yield_time >= 100)
            {
                vTaskDelay(1);
                last_yield_time = now;
            }
        }
        else
        {
            vTaskDelay(1); // this is important to allow the watchdog to be reset, and to let any other threads on the core work
        }
    }
}

extern "C" void app_main()
{
    
    initArduino(); //probably dont even need this
    xTaskCreateUniversal(loopTask, "loopTask", ARDUINO_LOOP_STACK_SIZE, NULL, 1, &loopTaskHandle, 1);
}
