/*
    Simple example on how to use PAA5100JE driver library.
    Works using interrupt pin (MOTION).
    Created by Matej Bozic,
    February, 2020.
    https://github.com/zic-95
*/

#include <Arduino.h>
#include <SPI.h>
#include "PAA5100JE.h"

// define SPI chip select pin
#define OF1_CS_PIN 9
// define interrupt motion_ready pin
#define OF1_MOTION_PIN 7
// define sensor reset pin (only used if NRESET pin is not connected to 3.3V)
#define OF1_RESET_PIN 5

#define PRINT_FREQ 100 // Define Serial print frequency in Hz

PAA5100JE_OF of1_sensor(OF1_CS_PIN);

unsigned long prev_update_time;
static uint32_t tTime;
double of1_working_height = 31.6; // in mm

// Storing values in counts
int16_t of1_delta_xy[2] = {0, 0};
int16_t of1_xy[2] = {0, 0};

volatile byte readMotion_flag = LOW;

void motionRead()
{
    readMotion_flag = HIGH;
}

void setup()
{
    Serial.begin(57600);
    SPI.begin();
    // Remove if NRESET pin is connected to 3.3V (e.g. in Pimoroni board)
    pinMode(OF1_RESET_PIN, OUTPUT);
    digitalWrite(OF1_RESET_PIN, HIGH);
    attachInterrupt(digitalPinToInterrupt(OF1_MOTION_PIN), motionRead, CHANGE);
    if (of1_sensor.init() == false)
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
    // Set X/Y axis orientation.
    of1_sensor.setOrientation(true, false, false);
    delay(5000);
    Serial.println("Test");
}
void loop()
{
    uint32_t t = millis();
    char serRead = Serial.read();
    // Reset values
    if (serRead == 'x')
    {
        of1_xy[0] = 0;
        of1_xy[1] = 0;
    }
    if (readMotion_flag == HIGH)
    {
        if (of1_sensor.motionRead(of1_delta_xy) == false)
            Serial.println("Error reading sensor values");
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
        readMotion_flag = LOW;
    }
    // Read values in normal mode (of1), burst mode (of2)
    if ((t - tTime) >= (1000 / PRINT_FREQ))
    {
        of1_xy[0] += of1_delta_xy[0];
        of1_xy[1] += of1_delta_xy[1];
        Serial.print("X1: ");
        Serial.print(of1_xy[0]);
        Serial.print(" Y1: ");
        Serial.print(of1_xy[1]);
        Serial.print("\n");
    }

    tTime = t;
}
