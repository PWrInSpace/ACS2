#ifndef __FLIGHTDATA__
#define __FLIGHTDATA__
#include <stdint.h>
#include <string>

#define STRUCT_SIZE 200 // subject to change
enum State{RAIL,ENGINE_FLIGHT,CONTROLLED_FLIGHT,FALL};
extern State state;

extern float P0;

#define SAMPLE_FREQUENCY 100 //in Hz
#define BUFFER_LENGTH 100 // subject to change
#define G 9.81 //gravitaional cceleration
#define AIR_DENSITY 2   

struct barometer
{
    float pressure;
    float altitude;   
};


union Data{

    struct
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

        float roll, pitch, yaw;// by magwick

        //Control surfaces

        float angle1, angle2;



        std::string toString()
        {
            std::string data = "";

            data += timeStamp;
            data += ",";
            data += state;
            data += ",";
            //data += ready; 
            //data += ",";

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

            data += roll;
            data += ",";
            data += pitch;
            data += ",";
            data += yaw;
            data += ",";


            data += angle1;
            data += ",";
            data += angle2;
            return data;
        }
    }values __attribute__((packed)); // will probably do this manually, 
                                      //so I know how to index the union
    uint8_t byte[STRUCT_SIZE];
}; 
#endif