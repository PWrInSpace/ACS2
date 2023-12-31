#ifndef __AGGREGATOR__
#define __AGGREGATOR__
#include "flightdata.h"
#include <MadgwickAHRS.h>
#include "freertos/FreeRTOS.h"

#include "time.h"
#include "flowcontroll.h"
#include "SPIFFS.h"
#include "FS.h"
#include "LITTLEFS.h"
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_L3GD20_U.h>
#include <ESP32Servo.h>
#include <BasicLinearAlgebra.h>

#undef MEMORY_SIZE
#define MEMORY_SIZE 1500
#define FIN_1_PIN 12
#define FIN_2_PIN 13

//
//RAM:   [=         ]   7.4% (used 24256 bytes from 327680 bytes)
//Flash: [===       ]  27.1% (used 354770 bytes from 1310720 bytes)I`


extern  Data RAM_Data[MEMORY_SIZE];
extern  unsigned  RAM_iter;
extern FlowControll fc;
extern int write_flag;
extern File file;

    static struct timespec start;
    static struct timespec finish;

    extern Adafruit_LSM303_Accel_Unified acc; 
    extern Adafruit_LSM303DLH_Mag_Unified mag;
    extern Adafruit_L3GD20_Unified gyro;
    extern State state;

    extern Servo fin_1; 
    extern Servo fin_2;

    extern FlowControll fc;

    extern Madgwick filter;
    
class ShiftReg
{
    //voodooo ooooo
    //hell awaits for creating this abomination

    //class for checking wheather a condition is met for a set number of evaluations
    private:
    uint32_t count; // up to 32 consecutive states
    uint8_t (*condition_func)(void*);//pointer to the condition which needs checking
    public:
    uint32_t reg_state;
    uint8_t check(void* params);
    ShiftReg(uint8_t (*condition)(void*), uint8_t n = 5);
};


class Tasks
{
 //TODO
 public:
    Tasks();
    static void ReadSensors(void* parameters);
    static void ReadBarometer(void* parameters);
    static void ProcessData(void* parameters);
    static void CalculateControlSignal(void* parameters);
    static void ServoController(void* parameters);
    static void WriteToFlash(void* parameters);
    static void StateMachine(void* parameters);
    static void FileRead(void* parameters);
};

    uint8_t check_engine_fiered(void* acceleration_norm);
    uint8_t check_engine_burnout(void* acceleration_norm);
    uint8_t check_decreasing_altitude(void* altitude);
    // I can no longer be bothered with the esoterics of c++ scoping
    extern ShiftReg launch_detector;
    extern ShiftReg burnout_detector;
    extern ShiftReg apogee_detector;

    State Rail(Data * data);
    State EngineFlight(Data * data);
    State ControlledFlight(Data *data);
    State Fall();

    bool ParachuteDeployed();



class GlobalAgreggator
{
private:
public:
    GlobalAgreggator();
    Tasks tasks;
};



uint8_t check_decreasing_altitude(void* altitude);

#endif
