#ifndef __FLIGHTDATA__
#define __FLIGHTDATA__
#include <stdint.h>
#include <string>

enum State{RAIL,ENGINE_FLIGHT,CONTROLLED_FLIGHT,FALL};
extern State state;

#define SAMPLE_FREQUENCY 100 //in Hz
struct barometer
{
    float pressure;
    float altitude;   
    float density;
};

struct servo_angles
{
    float angle1;
    float angle2;
};

union Data{

    struct data_struct
    {
    
        uint32_t timeStamp;
        uint8_t state;  // STATES: RAIL, ENGINE_FLIGHT, CONTROLLED_FLIGHT, FALL
        //bool ready; //Data ready for logging

        //raw sensor data
        //barometer:
        float pressure;
        //IMU:
        float accelX, accelY, accelZ;

        float gyroX, gyroY, gyroZ;

        float magX ,magY, magZ;

        //Processed data
        float altitude; // by integration and pressure readings

        //float roll, pitch, yaw;// by magwick

        //Control surfaces

        float angle1, angle2;

//        float dupa;


        std::string toString()
        {
            std::string data = "";

            data += timeStamp;
            data += ",";
            data += state;
            data += ",";

            data += pressure;
            data += ",";
            data += accelX;
            data += ",";
            data += accelY;
            data += ",";
            data += accelZ;
            data += ",";

            data += gyroX;
            data += ",";
            data += gyroY;
            data += ",";
            data += gyroZ;
            data += ",";

            data += magX;
            data += ",";
            data += magY;
            data += ",";
            data += magZ;
            data += ",";

            data += altitude; 
            data += ",";

            //data += roll;
            //data += ",";
            //data += pitch;
            //data += ",";
            //data += yaw;
            //data += ",";


            data += angle1;
            data += ",";
            data += angle2;
            return data;
        }
    }values __attribute__((packed));
                                    
    uint8_t byte[sizeof(data_struct)];
}; 
#endif