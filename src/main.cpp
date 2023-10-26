#include <Arduino.h>
#include "aggregator.h"
#include "flightdata.h"
#include "flowcontroll.h"
#include "freertos/FreeRTOS.h"

// #include "FS.h"
// #include "LittleFS.h"
//#include "SPIFFS.h"

//#define FORMAT_SPIFFS_IF_FAILED true


 GlobalAgreggator aggregator;
 FlowControll fc;

State state = RAIL;
  
//File file;
ShiftReg launch_detector(check_engine_fiered, 30);// later change to a value higher than 5
ShiftReg burnout_detector(check_engine_burnout, 30);
ShiftReg apogee_detector(check_decreasing_altitude,30);
Adafruit_LSM303_Accel_Unified acc;
Adafruit_LSM303DLH_Mag_Unified mag;
Adafruit_L3GD20_Unified gyro;

Servo fin_1;
Servo fin_2;

float bias_x;
float bias_y;
float bias_z;
 Data RAM_Data[MEMORY_SIZE];
 unsigned RAM_iter;
File file;
int write_flag=0;

void setup()
{


  //configure servos
	ESP32PWM::allocateTimer(0);
	fin_1.setPeriodHertz(50);
	fin_1.attach(FIN_1_PIN, 500, 2400);
	fin_2.setPeriodHertz(50);
	fin_2.attach(FIN_2_PIN, 500, 2400);

  write_flag=0;

  //doesnt mount without true, maybe mount when flight is detected?
  //while(1){Serial.println("Gówno");}
  
  /*if(!LITTLEFS.begin(true))
  {
    Serial.printf("An Error has occurred while mounting LittleFS");
    return;
  }*/




    Wire.setPins(8,9);
    acc = Adafruit_LSM303_Accel_Unified(54321);
    mag = Adafruit_LSM303DLH_Mag_Unified(12345);
    gyro = Adafruit_L3GD20_Unified(20);

    filter.begin(SAMPLE_FREQUENCY);
    vTaskDelay(pdMS_TO_TICKS(2000));
   //gyro.enableAutoRange(true);
  
  /* Initialise the sensor */
    gyro.enableAutoRange(false);
    if(!gyro.begin())
    {
      /* There was a problem detecting the L3GD20 ... check your connections */
      Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
      while(1);
    }
    Serial.println("Correcting biases");
    int samples = 1000;
    bias_x=bias_y=bias_z=0;
     sensors_event_t event;
    for(int i = 0;i<samples;++i)
    {
        gyro.getEvent(&event);
        bias_x+=event.gyro.x;
        bias_y+=event.gyro.y;
        bias_z+=event.gyro.z;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    bias_x/=samples;
    bias_y/=samples;
    bias_z/=samples;


    if (!acc.begin()) {
        /* There was a problem detecting the ADXL345 ... check your connections */
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while (1)
        ;
    }
    acc.setRange(LSM303_RANGE_8G);
    acc.setMode(LSM303_MODE_NORMAL);

    if (!mag.begin()) {
        /* There was a problem detecting the LSM303 ... check your connections */
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while (1)
        ;
    }
  
    fc.sensorQueue = xQueueCreate(2,sizeof(Data)); //for raw IMU and mag data
    fc.filterQueue = xQueueCreate(1,sizeof(Data)); //for processed data
    fc.controlQueue = xQueueCreate(1,sizeof(Data));//for control signals
    fc.normQueue = xQueueCreate(1,sizeof(Data));  // for acceleration norm
    fc.stateQueue = xQueueCreate(1,sizeof(Data));  // for acceleration norm
    fc.barometerQueue = xQueueCreate(1,sizeof(barometer));// for barometer readings; always overwritten due to different frequency

    if(fc.normQueue  == NULL || fc.sensorQueue  == NULL || 
       fc.filterQueue == NULL || fc.controlQueue == NULL ||
       fc.barometerQueue == NULL || fc.stateQueue == NULL)
    {
     Serial.printf("Queue creation failed");
    }

for(int i = 0; i < 5 ; i ++)
{
fin_1.write(90+i*6);
fin_2.write(90-i*6); 
vTaskDelay(pdMS_TO_TICKS(100));
}
for(int i = 0; i < 5 ; i ++)
{
fin_1.write(90-i*6);
fin_2.write(90+i*6); 
vTaskDelay(pdMS_TO_TICKS(100));
}

fin_1.write(90);
fin_2.write(80);

state = RAIL;

 Serial.println("READY!!!");
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
                
    xTaskCreate(aggregator.tasks.FileRead,
                "FileRead",
                2048,
                NULL,
                4,
                &fc.FileRead_handle);



/*
   vTaskDelay(pdMS_TO_TICKS(5000));
   state = CONTROLLED_FLIGHT;
   vTaskDelay(pdMS_TO_TICKS(10000));
   state = FALL;
   write_flag=1;
*/

   vTaskDelete(NULL);
}

void loop()
{ 
     vTaskDelete(NULL);

}