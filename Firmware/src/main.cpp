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

#include <Dps3xx.h>
#include <SPI.h>
#include "Config/pinmap_config.h"

SPIClass vspi;
// Dps3xx Opject
Dps3xx Dps3xxPressureSensor = Dps3xx();

void setup()
{
  //pin number of your slave select line
  //XMC2GO
  int16_t pin_cs = PinMap::BaroCs;
  //for XMC 1100 Bootkit  & XMC4700 Relax Kit uncomment the following line
  //int16_t pin_cs = 10;

  Serial.begin(9600);
  while (!Serial);

  vspi.begin(PinMap::H_SCLK,PinMap::H_MISO,PinMap::H_MOSI);


  //Call begin to initialize Dps3xxPressureSensor
  //The parameter pin_nr is the number of the CS pin on your Microcontroller
  Dps3xxPressureSensor.begin(vspi, pin_cs);

  Dps3xxPressureSensor.startMeasureBothCont(4,2,4,2);
  Dps3xxPressureSensor.disableFIFO();

  Serial.println("Init complete!");
}



void loop()
{
//   float temperature;
//   float pressure;
//   uint8_t oversampling = 7;
//   int16_t ret;
//   Serial.println();

  //lets the Dps3xx perform a Single temperature measurement with the last (or standard) configuration
  //The result will be written to the paramerter temperature
  //ret = Dps3xxPressureSensor.measureTempOnce(temperature);
  //the commented line below does exactly the same as the one above, but you can also config the precision
  //oversampling can be a value from 0 to 7
  //the Dps 3xx will perform 2^oversampling internal temperature measurements and combine them to one result with higher precision
  //measurements with higher precision take more time, consult datasheet for more information
//   ret = Dps3xxPressureSensor.measureTempOnce(temperature, oversampling);

 constexpr int PSR_B2 = 0x00;
    constexpr int RW_BYTE = 0x80;

    constexpr size_t numbytes = 6;
    //this method assumes the dps310 is in background mode
    std::array<uint8_t,numbytes> buffer;

    vspi.beginTransaction(SPISettings(DPS3xx__SPI_MAX_FREQ,
                                           MSBFIRST,
                                           SPI_MODE3));


    digitalWrite(PinMap::BaroCs,LOW);
    vspi.transfer(PSR_B2 | RW_BYTE); // start at PSR_B2
   

    for (int i = 0; i < numbytes; i++)
    {
        buffer[i] = vspi.transfer(0); //read out the 6 result registers
    }
    digitalWrite(PinMap::BaroCs,HIGH);
    vspi.endTransaction();

    int32_t raw_press = (uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2];
    int32_t raw_temp = (uint32_t)buffer[3] << 16 | (uint32_t)buffer[4] << 8 | (uint32_t)buffer[5];

    Dps3xxPressureSensor.getTwosComplement(&raw_press,24);
    Dps3xxPressureSensor.getTwosComplement(&raw_temp,24);

    // getRawResult(&raw_temp,dps::registerBlocks[dps::RegisterBlocks_e::TEMP]);
    // getRawResult(&raw_press,dps::registerBlocks[dps::RegisterBlocks_e::PRS]);
    // getRawResult(&raw_temp,dps::registerBlocks[dps::RegisterBlocks_e::TEMP]);

    float temperature = Dps3xxPressureSensor.calcTemp(raw_temp);
    float pressure = Dps3xxPressureSensor.calcPressure(raw_press);



Serial.print("Temperature: ");
Serial.print(temperature);
Serial.println(" degrees of Celsius");
Serial.print("Pressure: ");
Serial.print(pressure);
Serial.println(" Pascal");

 

  //Wait some time
 
}





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
