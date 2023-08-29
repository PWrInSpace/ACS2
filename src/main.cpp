#include <Arduino.h>
#include "aggregator.h"
#include "flightdata.h"
#include "flowcontroll.h"
#include "freertos/FreeRTOS.h"

#include "FS.h"
#include "LittleFS.h"
//#include "SPIFFS.h"

//#define FORMAT_SPIFFS_IF_FAILED true


 GlobalAgreggator aggregator;
 FlowControll fc;

State state = RAIL;
  
//File file;
ShiftReg launch_detector(check_engine_fiered, 5);// later change to a value higher than 5
ShiftReg burnout_detector(check_engine_burnout, 30);
ShiftReg apogee_detector(check_decreasing_altitude,30);
Adafruit_LSM303_Accel_Unified acc;
Adafruit_LSM303DLH_Mag_Unified mag;
Adafruit_L3GD20_Unified gyro;

void setup()
{
    vTaskDelay(pdMS_TO_TICKS(3000));
    Serial.begin(115200);


  //doesnt mount without true, maybe mount when flight is detected?

  if(!LittleFS.begin(true))
  {
    Serial.printf("An Error has occurred while mounting LittleFS");
    return;
  }



    Wire.setPins(8,9);
    acc = Adafruit_LSM303_Accel_Unified(54321);
    mag = Adafruit_LSM303DLH_Mag_Unified(12345);
    gyro = Adafruit_L3GD20_Unified(20);

    filter.begin(SAMPLE_FREQUENCY);

   //gyro.enableAutoRange(true);
  
  /* Initialise the sensor */
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }

    if (!acc.begin()) {
        /* There was a problem detecting the ADXL345 ... check your connections */
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while (1)
        ;
    }
    acc.setRange(LSM303_RANGE_8G);
    acc.setMode(LSM303_MODE_NORMAL);
    mag.enableAutoRange(true); 
    if (!mag.begin()) {
        /* There was a problem detecting the LSM303 ... check your connections */
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while (1)
        ;
    }
   
      
    vTaskDelay(pdMS_TO_TICKS(2000));
    fc.sensorQueue = xQueueCreate(2,sizeof(Data)); //for raw IMU and mag data
    fc.filterQueue = xQueueCreate(1,sizeof(Data)); //for processed data
    fc.controlQueue = xQueueCreate(1,sizeof(Data));//for control signals
    fc.normQueue = xQueueCreate(1,sizeof(float));  // for acceleration norm
    fc.stateQueue = xQueueCreate(1,sizeof(float));  // for acceleration norm
    fc.barometerQueue = xQueueCreate(1,sizeof(barometer));// for barometer readings; always overwritten due to different frequency

    if(fc.normQueue  == NULL || fc.sensorQueue  == NULL || 
       fc.filterQueue == NULL || fc.controlQueue == NULL ||
       fc.barometerQueue == NULL || fc.stateQueue == NULL)
    {
     Serial.printf("Queue creation failed");
    }




    vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskCreate( aggregator.tasks.ReadBarometer,
                "ReadBarometer",
                2048,
                NULL,
                2, // low priority due to low measurement frequency
                &fc.Barometer_handle);

    xTaskCreate( aggregator.tasks.ReadSensors,
                "ReadSensors",
                2048,
                NULL,
                configMAX_PRIORITIES - 1,//Sensor readings always take priority to keep the controll loop
                                         //functioning propperly
                &fc.Read_handle);

    xTaskCreate( aggregator.tasks.ProcessData,
                "FilterRawData",
                2048,
                NULL,
                5,
                &fc.Filter_handle);

    xTaskCreate( aggregator.tasks.CalculateControlSignal,
                "ControlSignal",
                2048,
                NULL,
                10,
                &fc.Control_handle);

    xTaskCreate( aggregator.tasks.WriteToFlash,
                "Write",
                2048,
                NULL,
                1,
                &fc.Write_handle);

    xTaskCreate(aggregator.tasks.StateMachine,
                "StateMachine",
                2048,
                NULL,
                4,
                &fc.State_machine_handle);


     
   vTaskDelete(NULL);
}

void loop()
{
  vTaskDelete(NULL);
}