/*
    Simple example on how to use PAA5100JE driver library.
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

#define OF_READ_FREQ 100 // Define reading frequency in Hz

// Set to 1 if you want to use in mm mode (sensor height must be defined), or to 0 if you want to use counts mode
#define USE_MM 1

PAA5100JE_OF of1_sensor(OF1_CS_PIN);
PAA5100JE_OF of2_sensor(OF2_CS_PIN);

unsigned long prev_update_time;
static uint32_t tTime;

double of1_working_height = 31.6; // in mm
double of2_working_height = 31.6; // in mm

#if USE_MM == 0
// Storing values in counts
int16_t of1_delta_xy[2] = {0, 0};
int16_t of2_delta_xy[2] = {0, 0};
int16_t of1_xy[2] = {0, 0};
int16_t of2_xy[2] = {0, 0};
#endif

#if USE_MM == 1
// Storing values in mm
double of1_delta_xy_mm[2] = {0.0, 0.0};
double of2_delta_xy_mm[2] = {0.0, 0.0};
double of1_xy_mm[2] = {0.0, 0.0};
double of2_xy_mm[2] = {0.0, 0.0};
#endif

void setup()
{
    Serial.begin(57600);
    SPI.begin();
    // Remove if NRESET pin is connected to 3.3V (e.g. in Pimoroni board)
    pinMode(OF1_RESET_PIN, OUTPUT);
    pinMode(OF2_RESET_PIN, OUTPUT);
    digitalWrite(OF1_RESET_PIN, HIGH);
    digitalWrite(OF2_RESET_PIN, HIGH);

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
}
void loop()
{
    uint32_t t = millis();
    char serRead = Serial.read();
    // Reset values
    if (serRead == 'x')
    {
#if USE_MM == 0
        of1_xy[0] = 0;
        of1_xy[1] = 0;
        of2_xy[0] = 0;
        of2_xy[1] = 0;

#endif
#if USE_MM == 1
        of1_xy_mm[0] = 0;
        of1_xy_mm[1] = 0;
        of2_xy_mm[0] = 0;
        of2_xy_mm[1] = 0;
#endif
    }
    // Read values in normal mode (of1), burst mode (of2)
    if ((t - tTime) >= (1000 / OF_READ_FREQ))
    {
#if USE_MM == 0
        if (of1_sensor.motionRead(of1_delta_xy) == false)
            Serial.println("Error reading sensor 1 values");
        else
        {
            of1_xy[0] += of1_delta_xy[0];
            of1_xy[1] += of1_delta_xy[1];
            Serial.print("X1: ");
            Serial.print(of1_xy[0]);
            Serial.print(" Y1: ");
            Serial.print(of1_xy[1]);
            Serial.print("\n");
        }
        if (of2_sensor.burstMotionRead(of2_delta_xy) == false)
            Serial.println("Error reading sensor 2 values");
        else
        {
            of2_xy[0] += of2_delta_xy[0];
            of2_xy[1] += of2_delta_xy[1];
            Serial.print("X2: ");
            Serial.print(of2_xy[0]);
            Serial.print(" Y2: ");
            Serial.print(of2_xy[1]);
            Serial.print("\n");
        }
#endif

#if USE_MM == 1
        if (of1_sensor.getDistance(of1_delta_xy_mm) == false)
            Serial.println("Error reading sensor 1 values");
        else
        {
            of1_xy_mm[0] += of1_delta_xy_mm[0];
            of1_xy_mm[1] += of1_delta_xy_mm[1];
            Serial.print("X1: ");
            Serial.print(of1_xy_mm[0]);
            Serial.print(" mm Y1: ");
            Serial.print(of1_xy_mm[1]);
            Serial.print(" mm\n");
        }
        if (of2_sensor.getDistance_burst(of2_delta_xy_mm) == false)
            Serial.println("Error reading sensor 2 values");
        else
        {
            of2_xy_mm[0] += of2_delta_xy_mm[0];
            of2_xy_mm[1] += of2_delta_xy_mm[1];
            Serial.print("X2: ");
            Serial.print(of2_xy_mm[0]);
            Serial.print("mm Y2: ");
            Serial.print(of2_xy_mm[1]);
            Serial.print(" mm\n");
        }
#endif

        tTime = t;
    }
}