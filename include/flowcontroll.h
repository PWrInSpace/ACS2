#ifndef __FLOWCONTROLL__
#define __FLOWCONTROLL__

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"




struct FlowControll{


  QueueHandle_t sensorQueue;
  QueueHandle_t filterQueue;
  QueueHandle_t controlQueue;
  QueueHandle_t barometerQueue;
  QueueHandle_t normQueue;
  QueueHandle_t stateQueue;
  QueueHandle_t servoQueue;

  TaskHandle_t Filter_handle;
  TaskHandle_t Read_handle;
  TaskHandle_t Control_handle;
  TaskHandle_t Write_handle;
  TaskHandle_t Barometer_handle;
  TaskHandle_t State_machine_handle;
  TaskHandle_t Servo_handle;

};

extern FlowControll fc;

#endif
