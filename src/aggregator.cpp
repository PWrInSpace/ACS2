#include "aggregator.h"
#include <Arduino.h>


float P0 = 1000;
Madgwick filter;

Loops::Loops()
{
}

State Loops::Rail()
{
    if(LaunchDetected())//check acceleration
    {
        //start recording data in a separate process

        return ENGINE_FLIGHT;   
    }
    else
        return RAIL;

}

State Loops::EngineFlight()
{
    if(EngineBurnout())//check if engine acceleration stops
    {
        //start controll loop

        return CONTROLLED_FLIGHT;
    }   
    else
        return ENGINE_FLIGHT;

}

State Loops::ControlledFlight()
{
    if(ParachuteDeployed())//check if altitude starts regularily  decreasing   
    {
        //stop recording data
        return FALL;
    }
    else
        return CONTROLLED_FLIGHT;
}

State Loops::Fall()
{
    return FALL;   
}

bool Loops::LaunchDetected()
{
    return false;
}

bool Loops::EngineBurnout()
{
    return false;
}

bool Loops::ParachuteDeployed()
{
    return false;
}

GlobalAgreggator::GlobalAgreggator()
{

}

Tasks::Tasks()
{
    clock_gettime( CLOCK_REALTIME, &start );
}

void Tasks::ReadSensors(void *parameters)
{

    TickType_t xLastWakeTime;
    BaseType_t xWasDelayed;
    Data tmp={};
    float temperature;


     sensors_event_t event;
    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount ();
    for(;;)
    {
        if(pdFALSE == (xWasDelayed = xTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(10) )))//strict 100Hz sampling frequency
            Serial.printf("RTOS can't keep up with current measurement frequency!!!\n\r");
        clock_gettime( CLOCK_REALTIME, &finish );
        tmp.values.timeStamp = finish.tv_sec*1000LL + finish.tv_nsec/1e6 - (start.tv_sec*1000LL + start.tv_nsec/1e6); // elapsed time in ms
        //some compiler fuckery prevents rand() from returning anything other than 0, 



        gyro.getEvent(&event);

        tmp.values.gyroX=event.gyro.x;
        tmp.values.gyroY=event.gyro.y;
        tmp.values.gyroZ=event.gyro.z;

        acc.getEvent(&event);

        tmp.values.accelX=event.acceleration.x;
        tmp.values.accelY=event.acceleration.y;
        tmp.values.accelZ=event.acceleration.z;

        mag.getEvent(&event);

        tmp.values.magX=event.magnetic.x;
        tmp.values.magY=event.magnetic.y;
        tmp.values.magZ=event.magnetic.z;

        tmp.values.state=state;

        if(xQueueSend(fc.sensorQueue, (void*)&tmp,0 ) != pdPASS )
        {
            fprintf(stderr,"Measurements not sent\n");
        }

    }

    vTaskDelete(NULL);
}

void Tasks::ProcessData(void *parameters)
{
    Data tmp;
    barometer bmp;
   while(true)
   {
        //if(xQueueReceive(sensorQueue, (void*)&tmp,pdMS_TO_TICKS(5*1000)) != pdPASS)
        if(xQueueReceive(fc.sensorQueue, (void*)&tmp,portMAX_DELAY) != pdPASS)// wait for queue to fill
        {
            fprintf(stderr,"Measurements not received\n");
            continue;
        }

        if(xQueuePeek(fc.barometerQueue, (void*)&bmp,portMAX_DELAY) != pdPASS)// since data is overwritten it should always be full
        {
            fprintf(stderr,"Barometer measurements not received\n");
            continue;
        }

        tmp.values.pressure=bmp.pressure;
        tmp.values.altitude=bmp.altitude;

        filter.update(  tmp.values.gyroY , -tmp.values.gyroX , tmp.values.gyroZ,
                        tmp.values.accelX, tmp.values.accelY, tmp.values.accelZ,
                        tmp.values.magX  , tmp.values.magY  , tmp.values.magZ 
                     );

        tmp.values.roll = filter.getRoll();
        tmp.values.pitch = filter.getPitch();
        tmp.values.yaw = filter.getYaw();

        //tmp.values.altitude = (P0-tmp.values.pressure )/(AIR_DENSITY*G);
    //will be corrected by a complementary filter with integrated acceleration
    //a matrix transformation is needed here to extract the vertical component
        if(xQueueSend(fc.filterQueue, (void*)&tmp, 0 ) != pdPASS)
        {
            fprintf(stderr,"State was not sent for processing\n");
        }


   }
    vTaskDelete(NULL);
}

void Tasks::CalculateControlSignal(void *parameters)
{
    Data tmp;
   while(true)
   {
        if(xQueueReceive(fc.filterQueue, (void*)&tmp,portMAX_DELAY) != pdPASS)
        {
            fprintf(stderr,"State was not received for processing\n");
            continue;
        }

        tmp.values.angle1 = tmp.values.pitch/2;
        tmp.values.angle2 = tmp.values.roll/3;

        if(xQueueSend(fc.controlQueue, (void*)&tmp, 0 ) != pdPASS)
        {
            fprintf(stderr,"Control was not sent for writing\n");
        }


   }
    vTaskDelete(NULL);

}

void Tasks::WriteToFlash(void *parameters)
{
    Data tmp;
    std::string string_data;
    float accel_len=0.0;
    for(;;)
    {
        if(xQueueReceive(fc.controlQueue, (void*) &tmp, portMAX_DELAY))
        {
         //   Serial.printf("Total: %u, Used: %u\n\r",LittleFS.totalBytes(),LittleFS.usedBytes());
            //string_data=tmp.values.toString();
        //  Serial.printf("Wrote: t - %d, pressure - %f, roll - %f, angle2 - %f\n\r",
        //                tmp.values.timeStamp,tmp.values.pressure ,tmp.values.roll,tmp.values.angle2);
        //Serial.printf("Ax - %3.3f, Ay - %3.3f, Az - %3.3f\n\r",tmp.values.accelX,tmp.values.accelY,tmp.values.accelZ);
        //  Serial.printf("%3.3f %3.3f %3.3f ",tmp.values.accelX,tmp.values.accelY,tmp.values.accelZ);
        //  Serial.printf("Mx - %f, My - %f, Mz - %f\n\r",tmp.values.magX,tmp.values.magY,tmp.values.magZ);
        //  Serial.printf("Gx - %f, Gy - %f, Gz - %f\n\r",tmp.values.gyroX,tmp.values.gyroY,tmp.values.gyroZ);
          //Serial.printf("%3.3f %3.3f %3.3f\n\r",tmp.values.gyroX,tmp.values.gyroY,tmp.values.gyroZ);
        //  Serial.printf("Roll - %f, Pitch - %f, Yaw - %f\n\r",tmp.values.roll,tmp.values.pitch,tmp.values.yaw);
    //    Serial.printf("%u %lu ",alt.check((void*)(&tmp.values.altitude)),alt.state);
    //    Serial.printf( "%3.3f %3.3f\n\r",tmp.values.pressure,tmp.values.altitude);
        //accel_len = sqrt(tmp.values.accelX*tmp.values.accelX + tmp.values.accelY*tmp.values.accelY + tmp.values.accelZ*tmp.values.accelZ);
        //Serial.printf("%3.3f  %3.3f  %3.3f  %3.3f\n\r",tmp.values.roll,tmp.values.pitch,tmp.values.yaw,accel_len);
                          
        }   
        else
            Serial.printf("GÓWNO WYJEBALO");
    }

    vTaskDelete(NULL);
}


void Tasks::ReadBarometer(void *parameters)
{

    TickType_t xLastWakeTime;
    BaseType_t xWasDelayed;
    Adafruit_BMP085 bmp ;
    barometer data;
    
 
    if (!bmp.begin(BMP085_ULTRALOWPOWER)) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        while (1) {}
    }
     sensors_event_t event;
    // Initialise the xLastWakeTime variable with the current time.
    ShiftReg alt(check_decreasing_altitude,10);
    xLastWakeTime = xTaskGetTickCount ();

    for(;;)
    {
        if(pdFALSE == (xWasDelayed = xTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(30) )))//dumb fix because pressure measurements took too long
            Serial.printf("RTOS can't keep up with current barometer measurement frequency!!!\n\r");

        data.pressure = bmp.readPressure();
        data.altitude = 44330 * (1.0 - pow(data.pressure / 101325 , 0.1903));// 101325 is the standard sea level pressure
        Serial.printf("%u %lu ",alt.check((void*)(&data.altitude)),alt.state);
        Serial.printf( "%3.3f %3.3f\n\r",data.pressure,data.altitude);

        if(xQueueOverwrite(fc.barometerQueue, (void*)&data ) != pdPASS )
        {
            fprintf(stderr,"Barometer measurements not sent\n");
        }
    }

    vTaskDelete(NULL);
}


ShiftReg::ShiftReg(uint8_t (*condition)(void*), uint32_t n)
{
    condition_func=condition;
    count = (-1) << n-1;
    state = 0;
}

uint8_t ShiftReg::check(void* params)
{
    state = (state<<1) | !condition_func(params) | (count<<1);
    return (state == count);
}

uint8_t check_decreasing_altitude(void* altitude)
{
    //because the barometer measurements are done at a different frequency and 
    //this function is executed at the base 100Hz, multiple samples have to be checked, because they can
    //remain constant during the main sampling period(up to 3)

    //alternatively, this state change check can be performed at this lower frequency, 
    //because it's not as important as taking measurements.

    //this latter approach works better, but the parameters have to be tweeked in accordance to the falling
    //rocket speed, eg. the fewer checks have to be performed the sooner the apoapsis will be caught,
    //but it can lead to false positives

    //might need to average a large number of samples, 
    //pressure changes to slowly for this method to register

    static float alt_prev = 0;
    float current_alt = *(float*)altitude;

    if((current_alt - alt_prev )<= 0)
    {
        alt_prev = current_alt;
        return 1  ;
    }
    else
    {
        alt_prev = current_alt;
        return 0;
    }
}