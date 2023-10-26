#include "aggregator.h"
#include <Arduino.h>

Madgwick filter;

extern float bias_x;
extern float bias_y;
extern float bias_z;


State Rail(Data *data)
{
        float acc_norm = sqrt(data->values.accelX*data->values.accelX + data->values.accelY*data->values.accelY + data->values.accelZ*data->values.accelZ);
    if(data->values.altitude > 3 && launch_detector.check((void*)(&acc_norm)))// at least 2 meters off the ground to prevent accidentaly chaning state due to bumping
    {
        //start recording data in a separate process

        return ENGINE_FLIGHT;   
    }
    else
        return RAIL;

}

State EngineFlight(Data *data)
{
        float acc_norm = sqrt(data->values.accelX*data->values.accelX + data->values.accelY*data->values.accelY + data->values.accelZ*data->values.accelZ);
    if(burnout_detector.check((void*)(&acc_norm)))
    {
        //start controll loop

        return CONTROLLED_FLIGHT;
    }   
    else
        return ENGINE_FLIGHT;

}

State ControlledFlight(Data *data)
{

    if(apogee_detector.check((void*)(&data->values.altitude)))//check if altitude starts regularily  decreasing   
    {
        //stop recording data
        write_flag = 1;
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
//
//    float x_gyro_bias=0;
//    float y_gyro_bias=0;
//    float z_gyro_bias=0;
//    //bias calculation
//    const uint16_t samples = 1000;
//    for(int i = 0;i<samples;++i)
//    {
//        gyro.getEvent(&event);
//        x_gyro_bias+=event.gyro.x;
//        y_gyro_bias+=event.gyro.y;
//        z_gyro_bias+=event.gyro.z;
//    }
//    x_gyro_bias/=samples;
//    y_gyro_bias/=samples;
//    z_gyro_bias/=samples;
//    bias = z_gyro_bias;
//
//    float ang_x=0;
//
//float vel_ang_x_p = 0;
    for(;;)
    {
        if(pdFALSE == (xWasDelayed = xTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(10) )))//strict 100Hz sampling frequency
            //Serial.printf("RTOS can't keep up with current measurement frequency!!!\n\r");
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
    Data tmp;
    for(;;)
    {
        if(xQueueReceive(fc.stateQueue, (void*)&tmp,portMAX_DELAY) != pdPASS)// wait for queue to fill
        {
            fprintf(stderr,"Measurements not received\n");
            continue;
        }
        //Serial.printf("%u  %3.3f\n\r",state,data);
        state = ControlledFlight(&tmp);
        break;
    }
    vTaskDelete(NULL);
}

void Tasks::ProcessData(void *parameters)
{
    Data tmp;
    barometer bmp;
 
   float norm;
   float angle = 0;
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

        if(RAIL == state || ENGINE_FLIGHT == state)
        {
         if(xQueueSend(fc.stateQueue, (void*)&tmp ,0) != pdPASS )
         {
             fprintf(stderr,"flight data not sent to state machine\n");
         }
        }

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
    servo_angles servo;
    float old_vel=0;
    float new_vel=0;

    float ALPHA =0.99;
    float old_alt =0;
    float angle;
    const float a = 23.0;// from matlab
    const float b = 3.0;
    float u=0;
    float vel = 25;
    float acc_norm=0;
   while(true)
   {
        if(xQueueReceive(fc.filterQueue, (void*)&tmp,portMAX_DELAY) != pdPASS)
        {
            fprintf(stderr,"State was not received for processing\n");
            continue;
        }
        //integrate axis
        angle +=(tmp.values.gyroZ-bias_z)*0.01;

        //calculate vertical velocity
        old_vel=new_vel;

        new_vel = 100.0*(tmp.values.altitude-old_alt);
        new_vel = ALPHA*old_vel+(1-ALPHA)*new_vel;

        if(CONTROLLED_FLIGHT == state){
                u = -a*angle-b*(tmp.values.gyroZ-bias_z);
            if(vel > 1)
                u/=vel*vel;//velocity must be positive
            else 
                u = 0; // avoid zero singularity
            //u = asinf(u)/6;//we aproximate and try to only operate before the lift coefficient stalls at around 15 degs
                u/=6;
                u*=180/3.1415;
                servo.angle1=u;
                tmp.values.angle1=u;
                servo.angle2=u;
                tmp.values.angle2=-u;
                fin_1.write(servo.angle1+90); 
                fin_2.write(servo.angle2 +80); 
                
        }
        
        new_vel = ALPHA*new_vel + (1-ALPHA)*100*(tmp.values.altitude-old_alt);
        old_alt = tmp.values.altitude;
//velocity's fine, assume constant density
        acc_norm = sqrt(tmp.values.accelX*tmp.values.accelX + tmp.values.accelY*tmp.values.accelY + tmp.values.accelZ*tmp.values.accelZ);
        //Serial.printf("%u %3.3f %3.3f  %3.3f %3.3f %3.3f\n\r",state, u, angle,acc_norm,tmp.values.dupa,tmp.values.altitude);
        if (FALL != state)
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
    float angle=0;
  
 
    unsigned i,j;
    /*for(;;)
    {
        if(state == ENGINE_FLIGHT || state == CONTROLLED_FLIGHT)
        {
        if(xQueueReceive(fc.controlQueue, (void*) &tmp, portMAX_DELAY))
        {
            if(RAM_iter<MEMORY_SIZE)
                          RAM_Data[RAM_iter++] = tmp;
        
        }   
        else
            Serial.printf("GÓWNO WYJEBALO");
        }
        if(state == FALL)
        {
            if(write_flag)
            {
                 file = LITTLEFS.open("/altimetry", FILE_WRITE);
                 Serial.printf("writing to file\n");
                for(i = 0; i< MEMORY_SIZE;++i )
                {
                        file.write((uint8_t*)RAM_Data[i].byte,sizeof(Data));
                }
                file.close();
                 //file = LITTLEFS.open("/altimetry", FILE_READ);
      
                write_flag=0;
            }
            else
            { 
              
                
            }
        }
    }*/

    vTaskDelete(NULL);
}

void Tasks::FileRead(void* parameters)
{
    pinMode(0, INPUT_PULLUP); 
    Data tmp;
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if(! digitalRead(0)&& state == RAIL)
        {
              file = LITTLEFS.open("/altimetry", FILE_READ);
                for(int i = 0; i < MEMORY_SIZE; ++i)
                {
                    file.readBytes((char*)tmp.byte,sizeof(Data));
                    Serial.printf("%lu %u %f %f %f %f %f %f %f %f %f %f %f %f\n\r", tmp.values.timeStamp, tmp.values.state,
                                    tmp.values.accelX, tmp.values.accelY, tmp.values.accelZ,
                                    tmp.values.gyroX, tmp.values.gyroY, tmp.values.gyroZ,
                                    tmp.values.magX, tmp.values.magY, tmp.values.magZ,
                                    tmp.values.angle1, tmp.values.angle2);
                }       
                file.close();
                
        }
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
    Data jestemnagranicy;

    if (!bmp.begin(BMP085_ULTRALOWPOWER)) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        while (1) {}
    }
    // Initialise the xLastWakeTime variable with the current time.
    

    xLastWakeTime = xTaskGetTickCount ();
    float initial_pressure = bmp.readPressure().pressure;
    float initial_temperature = bmp.readTemperature();
    float initial_density=initial_pressure/(287.058*(initial_temperature+273.15));

    kurwamac japierdole;
    for(;;)
    {
        if(pdFALSE == (xWasDelayed = xTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(32) )))//dumb fix because pressure measurements took too long
          //  Serial.printf("RTOS can't keep up with current barometer measurement frequency!!!\n\r");

        japierdole = bmp.readPressure();
        data.pressure=japierdole.pressure;
        old_alt = new_alt;
        new_alt = (initial_temperature+273.15)/0.0065*(1.0 - pow(data.pressure / initial_pressure , 0.1903));//borrowed from apogemix
        new_alt = ALPHA*old_alt + (1-ALPHA)*new_alt;
        data.density= japierdole.pressure/(287.058*(japierdole.temperature+273.15));
        data.altitude = new_alt;
        jestemnagranicy.values.altitude=data.altitude;
        if(CONTROLLED_FLIGHT == state)
        {
         if(xQueueSend(fc.stateQueue, (void*)&jestemnagranicy ,0) != pdPASS )// 
         //if(xQueueSend(fc.stateQueue, (void*)&new_speed ,0) != pdPASS )
         {
             Serial.printf("altitude not sent to state machine\n");
         }
        }


        if(xQueueOverwrite(fc.barometerQueue, (void*)&data ) != pdPASS )
        {
            Serial.printf("Barometer measurements not sent\n");
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

    if(current_alt +3< alt_max)//jebac
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
    if(norm > 30)
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

    if(norm < 15)
    {
        return 1;
    }
    else
    {
        return 0;
    }

}