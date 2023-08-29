#include "aggregator.h"
#include <Arduino.h>


Madgwick filter;


State Rail(float acc_norm)
{

    if(launch_detector.check((void*)(&acc_norm)))
    {
        //start recording data in a separate process

        return ENGINE_FLIGHT;   
    }
    else
        return RAIL;

}

State EngineFlight(float acc_norm)
{
    if(burnout_detector.check((void*)(&acc_norm)))
    {
        //start controll loop

        return CONTROLLED_FLIGHT;
    }   
    else
        return ENGINE_FLIGHT;

}

State ControlledFlight(float alt)
{

    if(apogee_detector.check((void*)(&alt)))//check if altitude starts regularily  decreasing   
    {
        //stop recording data
        return FALL;
    }
    else
        return CONTROLLED_FLIGHT;
}

State Fall()
{
    return FALL;   
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

void Tasks::StateMachine(void *parameters)
{
    float data=0;
    for(;;)
    {
        if(xQueueReceive(fc.stateQueue, (void*)&data,portMAX_DELAY) != pdPASS)// wait for queue to fill
        {
            fprintf(stderr,"Measurements not received\n");
            continue;
        }

  
      switch (state)
      {
  
        case RAIL: // rocket waiting for launch, no data logging
                   // launch detected intertially
          state = Rail(data);
          break;
        case ENGINE_FLIGHT: // engine on, control loop off
          state = EngineFlight(data);
          break;
        case CONTROLLED_FLIGHT: //engine off, control loop on

        Serial.printf("%u  %3.3f\n\r",state,data);
          state = ControlledFlight(data);
          break;
        case FALL: //apogee achieved, no more data collection, control loop off
          vTaskDelete(NULL); //no more state changes
          //state = Fall();
          break;
        default:
          break;
    
        }
    }
    vTaskDelete(NULL);
}

void Tasks::ProcessData(void *parameters)
{
    Data tmp;
    barometer bmp;
    float norm;
   while(true)
   {
        
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
        if(RAIL == state || ENGINE_FLIGHT == state)
        {
        norm = sqrt(tmp.values.accelX*tmp.values.accelX + tmp.values.accelY*tmp.values.accelY + tmp.values.accelZ*tmp.values.accelZ);
         if(xQueueSend(fc.stateQueue, (void*)&norm ,0) != pdPASS )
         {
             fprintf(stderr,"flight data not sent to state machine\n");
         }
        }

        //tmp.values.altitude = (P0-tmp.values.pressure )/(AIR_DENSITY*G);
    //will be corrected by a complementary filter with integrated acceleration
    //a matrix transformation is needed here to extract the vertical component
        if(xQueueSend(fc.filterQueue, (void*)&tmp, 0 ) != pdPASS)
        {
            fprintf(stderr,"flight data not sent for processing\n");
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
    Data tmp={0};
    std::string string_data;
    float accel_len=0.0;
    float old_alt=0.0;
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
        accel_len = sqrt(tmp.values.accelX*tmp.values.accelX + tmp.values.accelY*tmp.values.accelY + tmp.values.accelZ*tmp.values.accelZ);
        //Serial.printf("%u  %3.3f  %3.3f  %3.3f\n\r",state,tmp.values.altitude,tmp.values.diff);
                          
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
    const float ALPHA = 0.2;
    
    float new_alt = 0;
    float old_alt = 0;
    if (!bmp.begin(BMP085_ULTRALOWPOWER)) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        while (1) {}
    }
    // Initialise the xLastWakeTime variable with the current time.
    

    xLastWakeTime = xTaskGetTickCount ();
    float initial_pressure = bmp.readPressure();
    float initial_temperature = bmp.readTemperature();
    for(;;)
    {
        if(pdFALSE == (xWasDelayed = xTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(30) )))//dumb fix because pressure measurements took too long
            Serial.printf("RTOS can't keep up with current barometer measurement frequency!!!\n\r");

        data.pressure = bmp.readPressure();
        //data.altitude = 44330 * (1.0 - pow(data.pressure / 101325 , 0.1903));// 101325 is the standard sea level pressure
        old_alt = new_alt;
        //new_alt = 44330 * (1.0 - pow(data.pressure / 101325 , 0.1903));// 101325 is the standard sea level pressure
        new_alt = (initial_temperature+273.15)/0.0065*(1.0 - pow(data.pressure / initial_pressure , 0.1903));//borrowed from apogemix
        new_alt = ALPHA*old_alt + (1-ALPHA)*new_alt;
        data.altitude = new_alt;

        if(CONTROLLED_FLIGHT == state)
        {
         if(xQueueSend(fc.stateQueue, (void*)&data.altitude ,0) != pdPASS )// 
         //if(xQueueSend(fc.stateQueue, (void*)&new_speed ,0) != pdPASS )
         {
             fprintf(stderr,"altitude not sent to state machine\n");
         }
        }


        if(xQueueOverwrite(fc.barometerQueue, (void*)&data ) != pdPASS )
        {
            fprintf(stderr,"Barometer measurements not sent\n");
        }
    }

    vTaskDelete(NULL);
}


ShiftReg::ShiftReg(uint8_t (*condition)(void*), uint8_t n)
{
    condition_func=condition;
    n = n > 31 ? 31 : n;
    count = (-1) << n-1;
    reg_state = 0;
}

uint8_t ShiftReg::check(void* params)
{
    reg_state = (reg_state<<1) | !condition_func(params) | (count<<1);
    return (reg_state == count);
}

uint8_t check_decreasing_altitude(void* altitude)
{
    static float alt_max = 0;
    float current_alt = *(float*)altitude;

    if(current_alt +1< alt_max)//jebac
    {
        return 1;
    }
    else
    {
        alt_max = current_alt;
        return 0;
    }
}

uint8_t check_engine_fiered(void* acceleration_norm)
{
    float norm = *(float*)acceleration_norm;
    //max acceleration should be around 6.6g
    // around 5.5g should be safe enough for the fiering threshold
    if(norm > 55)
    {
        return 1;   
    }
    else
    {
        return 0;
    }

}

uint8_t check_engine_burnout(void* acceleration_norm)
{
    float norm = *(float*)acceleration_norm;
    //accelerometer at reast reads a norm of around 8.5
    // threshold of 10 should be good enough, considering that drag
    // will sum with the gravity vector

    if(norm < 10)
    {
        return 1;
    }
    else
    {
        return 0;
    }

}