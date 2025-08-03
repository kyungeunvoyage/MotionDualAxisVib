/*
 Name:		MotionOrthoVib03.ino
 Created:	2025-08-03 오전 11:41:25
 Author:	HCITECH_01
*/

// the setup function runs once when you press reset or power the board
#include <Wire.h>
#include "Haptic_Driver.h"
#include <Audio.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>

//================DA7280====================
#define NUMBER_OF_SENSORS 2
Haptic_Driver hapDrive[NUMBER_OF_SENSORS];
hapticSettings hf;
#define MUX_ADDR 0x70
bool daInitialized[NUMBER_OF_SENSORS];


int burst75ms = 75;
int burst200ms = 200;
int burst500ms = 500;

float currentFreq = 10.0;
bool vibrating = false;
unsigned long vibStart = 0;
unsigned long vibDuration = 0;
//================DA7280====================

//================DA7280 setup====================

void DA7280setup()
{
    hf.motorType = LRA_TYPE;
    hf.absVolt = 3.5;
    hf.nomVolt = 2.47;
    hf.currMax = 295.1;
    hf.impedance = 8.37;
    //init frequency 
    hf.lraFreq = 5;
    Wire.begin();

    for (int i = 0; i < NUMBER_OF_SENSORS; i++) {
        //TCA9548A(i);
        hapDrive[i].begin();
        daInitialized[i] = true;
        if (!hapDrive[i].begin()) {
            Serial.print("Could not communicate with Haptic Driver ");
            Serial.println(i);
        }
        hapDrive[i].enableFreqTrack(false);
        hapDrive[i].setOperationMode(DRO_MODE);
    }
}
//================DA7280 setup====================

//===========Arrange DA7280=======================
void soleDA7280(Haptic_Driver& driver, uint8_t channel, int duration, int intensity, float freq) {
    driver.setActuatorLRAfreq(freq);
    driver.setVibrate(intensity);
    vibStart = millis();
    vibDuration = duration;
    vibrating = true;
    Serial.print("[VIBRATE] Channel ");
    Serial.print(channel);
    Serial.print(" - ");
    Serial.print(duration);
    Serial.println(" ms");
    delay(2000);
}

void setup() {
    
    Serial.begin(115200);
    DA7280setup();
    Serial.println("Hello Helllo AND Initiated");
    
}

// the loop function runs over and over again until power down or reset
void loop() 
{
    if (Serial.available() > 0)
    {
        char command = Serial.read();

        if (command == 'A')
        {
            //activate DA7280 
            soleDA7280(hapDrive[0], 0, burst75ms, 100, 175.0);
        }
    }

    
}
