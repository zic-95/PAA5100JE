/*
    PAA5100JE has Frame synchronization feauture.
    It can be synched with multiple PAA5100JE sensors,
    or other sensors (e.g. IMU).
    Works in polling mode, reading at set frequency.
    Created by Matej Bozic,
    February, 2020.
    https://github.com/zic-95
*/

#include <Arduino.h>
#include <SPI.h>
#include "PAA5100JE.h"

// define SPI chip select pin
#define OF1_CS_PIN 9
#define OF2_CS_PIN 8
// define interrupt motion_ready pin
#define OF1_MOTION_PIN 7
#define OF2_MOTION_PIN 6
// define sensor reset pin (only used if NRESET pin is not connected to 3.3V)
#define OF1_RESET_PIN 5
#define OF2_RESET_PIN 4

#define OF_PRINT_FREQ 10 // Define reading frequency in Hz

// Timer perios
#define FPS_242 4.2  // 4.2 ms or 242 FPS
#define FPS_121 8.3  // 8.3 ms or 121 FPS
#define FPS_100 10.0 // 10 ms or 100 FPS
#define FPS_90 11.1  // 11.1 ms or 90 FPS
#define FPS_80 12.5  // 12.5 ms or 80 FPS

PAA5100JE_OF of1_sensor(OF1_CS_PIN);
PAA5100JE_OF of2_sensor(OF2_CS_PIN);

unsigned long prev_update_time;
static uint32_t tTime_print;
static uint32_t frame_timer;

double of1_working_height = 31.6; // in mm
double of2_working_height = 31.6; // in mm

// Storing values in counts
int16_t of1_delta_xy[2] = {0, 0};
int16_t of2_delta_xy[2] = {0, 0};
int16_t of1_xy[2] = {0, 0};
int16_t of2_xy[2] = {0, 0};

byte writeFrameSyncAdr_2[4] = {0x7F, 0x15, 0x7F, 0x40};
byte writeFrameSyncVal_2[4] = {0x00, 0x00, 0x07, 0x40};
byte writeFrameSyncAdr_3[2] = {0x40, 0x7F};
byte writeFrameSyncVal_3[2] = {0x41, 0x00};

void syncFrameReadProcedure();

void setup()
{
    Serial.begin(57600);
    SPI.begin();
    // Remove if NRESET pin is connected to 3.3V (e.g. in Pimoroni board)
    pinMode(OF1_RESET_PIN, OUTPUT);
    pinMode(OF2_RESET_PIN, OUTPUT);
    digitalWrite(OF1_RESET_PIN, HIGH);
    digitalWrite(OF2_RESET_PIN, HIGH);
    //... other Optical Flow sensors

    if (of1_sensor.init() == false || of2_sensor.init() == false)
    {
        while (true)
        {
            Serial.println("Error in sensors initialization");
        }
    }
    delay(100);
    // Set sensor working height first.
    // This can be done realtime using distance sensor, if sensor/surface height is changing.
    of1_sensor.setWorkingHeight(of1_working_height);
    of2_sensor.setWorkingHeight(of2_working_height);
    // Set X/Y axis orientation.
    of1_sensor.setOrientation(true, false, false);
    of2_sensor.setOrientation(true, false, false);
    delay(100);

    of1_sensor.frameSync_stopOperation();
    of2_sensor.frameSync_stopOperation();
    delay(10);
}
void loop()
{
    uint32_t t = millis();
    frame_timer = t;
    byte flagSet = 0;
    while ((frame_timer - t) <= FPS_100)
    {
        // Start the frame synchronization procedure
        syncFrameReadProcedure();
        // Read motion from Optical Flow sensors
        // The maximum period to read motion data for all
        // the chips should not exceed 0.6 ms (regardless of
        // timer period used). Use burstMotionRead() if you
        // want to speed up read process.
        if (of1_sensor.motionRead(of1_delta_xy) == false)
            Serial.println("Error reading sensor 1 values");
        else
        {
            of1_xy[0] += of1_delta_xy[0];
            of1_xy[1] += of1_delta_xy[1];
        }
        if (of2_sensor.motionRead(of2_delta_xy) == false)
            Serial.println("Error reading sensor 1 values");
        else
        {
            of2_xy[0] += of2_delta_xy[0];
            of2_xy[1] += of2_delta_xy[1];
        }
        //... other Optical Flow sensors
        //... read IMU sensor etc.

        frame_timer = millis();
    }
    // Check if bit 5 is set from Observation register
    if ((of1_sensor.getObservation() & 0x20) != 0x20)
    {
        delay(1);
        for (int i = 0; i < 3; i++)
        {
            if ((of1_sensor.getObservation() & 0x20) == 0x20)
            {
                flagSet = 1;
                break;
            }
        }
        if (flagSet != 1)
        {
            while (1)
            {
                // Do the restart procedure
                Serial.println("Error reading OF1 frame sync routine.");
            }
        }
    }
    flagSet = 0;
    if ((of2_sensor.getObservation() & 0x20) != 0x20)
    {
        delay(1);
        for (int i = 0; i < 3; i++)
        {
            if ((of2_sensor.getObservation() & 0x20) == 0x20)
            {
                flagSet = 1;
                break;
            }
        }
        if (flagSet != 1)
        {
            while (1)
            {
                // Do the restart procedure
                Serial.println("Error reading OF2 frame sync routine.");
            }
        }
    }

    if ((t - tTime_print) >= OF_PRINT_FREQ)
    {
        Serial.print("X1: ");
        Serial.print(of1_xy[0]);
        Serial.print(" Y1: ");
        Serial.print(of1_xy[1]);
        Serial.print("\n");
        Serial.print("X2: ");
        Serial.print(of2_xy[0]);
        Serial.print(" Y2: ");
        Serial.print(of2_xy[1]);
        Serial.print("\n");
        tTime_print = t;
        //... other Optical Flow sensors
    }
}

void syncFrameReadProcedure()
{

    for (int i = 0; i < 4; i++)
    {
        SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
        // Set all OF CS pins low to write to all OF sensors registers at
        //  the same time.
        digitalWrite(OF1_CS_PIN, LOW);
        digitalWrite(OF2_CS_PIN, LOW);
        // digitalWrite(OF3_CS_PIN, LOW);
        // etc...
        delayMicroseconds(1);
        SPI.transfer(writeFrameSyncAdr_2[i]);
        SPI.transfer(writeFrameSyncVal_2[i]);
        delayMicroseconds(5);
        digitalWrite(OF1_CS_PIN, HIGH);
        digitalWrite(OF2_CS_PIN, HIGH);
        // digitalWrite(OF3_CS_PIN, HIGH);
        // etc...
        SPI.endTransaction();
        delayMicroseconds(5);
    }
    delayMicroseconds(430);
    for (int i = 0; i < 2; i++)
    {
        SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
        digitalWrite(OF1_CS_PIN, LOW);
        digitalWrite(OF2_CS_PIN, LOW);
        // digitalWrite(OF3_CS_PIN, LOW);
        // etc...
        delayMicroseconds(1);
        SPI.transfer(writeFrameSyncAdr_3[i]);
        SPI.transfer(writeFrameSyncVal_3[i]);
        delayMicroseconds(5);
        digitalWrite(OF1_CS_PIN, HIGH);
        digitalWrite(OF2_CS_PIN, HIGH);
        // digitalWrite(OF3_CS_PIN, HIGH);
        // etc...
        SPI.endTransaction();
        delayMicroseconds(5);
    }
}
