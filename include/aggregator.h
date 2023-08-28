#ifndef __AGGREGATOR__
#define __AGGREGATOR__
#include "flightdata.h"
#include <MadgwickAHRS.h>
#include "freertos/FreeRTOS.h"

#include "time.h"
#include "flowcontroll.h"
#include "LittleFS.h"
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_L3GD20_U.h>
    static struct timespec start;
    static struct timespec finish;

    extern Adafruit_LSM303_Accel_Unified acc; 
    extern Adafruit_LSM303DLH_Mag_Unified mag;
    extern Adafruit_L3GD20_Unified gyro;

    extern FlowControll fc;

    extern Madgwick filter;
    
class ShiftReg
{
    //voodooo ooooo
    //hell awaits for creating this abomination

    //class for checking wheather a condition is met for a set number of evaluations
    private:
    uint32_t count; // up to 32 
    uint8_t (*condition_func)(void*);//pointer to the condition which needs checking
    public:
    uint32_t state;
    uint8_t check(void* params);
    ShiftReg(uint8_t (*condition)(void*), uint32_t n = 5);
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
    static void WriteToFlash(void* parameters);
};

class Loops
{
 //TODO
public:
    Loops();
   
    State Rail();
    State EngineFlight();
    State ControlledFlight();
    State Fall();
private:
    bool LaunchDetected();
    bool EngineBurnout();
    bool ParachuteDeployed();
};



class GlobalAgreggator
{
private:
public:
    GlobalAgreggator();
    Loops loops; // state dependant loops
    Tasks tasks;
};



uint8_t check_decreasing_altitude(void* altitude);

#endif
