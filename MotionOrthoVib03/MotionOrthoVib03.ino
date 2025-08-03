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



//==================titanLF setup====================
const int DAC_PIN_A21 = A21; //Teensy 3.5 DAC0 == L in
const int DAC_PIN_A22 = A22; //Teensy 3.5 DAC1 == R in

const int waveformSize = 256;
int waveformIndex = 0;
float targetHz = 40.0;
float cycleDurationMs = 1000.0 / targetHz;
float delayPerSampleUs = (cycleDurationMs * 1000.0) / waveformSize;
float delayPerSampleUs_rt = (cycleDurationMs * 1000.0) / waveformSize;
int16_t negDat[256];
int16_t negDatTrial[256];

int16_t dat[waveformSize] = {
-19754, -19235, -18393, -17264, -15891,
-14325, -12622, -10836, -9021, -7223, -5485, -3840, -2314, -924, 317, 1405,
2341, 3129, 3776, 4293, 4692, 4983, 5180, 5295, 5339, 5322, 5255, 5147, 5006,
4838, 4651, 4449, 4236, 4018, 3796, 3574, 3355, 3139, 2929, 2725, 2528, 2339,
2159, 1987, 1825, 1671, 1527, 1391, 1265, 1146, 1036, 935, 841, 754, 675, 602,
537, 477, 424, 376, 334, 296, 264, 236, 213, 193, 178, 166, 157, 152, 149, 149,
152, 157, 164, 174, 185, 198, 212, 228, 246, 264, 283, 304, 325, 347, 369, 392,
415, 439, 463, 487, 511, 536, 560, 584, 608, 632, 655, 678, 701, 723, 745, 766,
787, 807, 826, 845, 863, 881, 897, 913, 928, 943, 956, 969, 980, 991, 1001, 1010,
1018, 1026, 1032, 1037, 1041, 1045, 1047, 1049, 1049, 1049, 1047, 1045, 1041, 1037,
1032, 1026, 1018, 1010, 1001, 991, 980, 969, 956, 943, 928, 913, 897, 881, 863, 845,
826, 807, 787, 766, 745, 723, 701, 678, 655, 632, 608, 584, 560, 536, 511, 487, 463,
439, 415, 392, 369, 347, 325, 304, 283, 264, 246, 228, 212, 198, 185, 174, 164, 157,
152, 149, 149, 152, 157, 166, 178, 193, 213, 236, 264, 296, 334, 376, 424, 477, 537,
602, 675, 754, 841, 935, 1036, 1146, 1265, 1391, 1527, 1671, 1825, 1987, 2159, 2339,
2528, 2725, 2929, 3139, 3355, 3574, 3796, 4018, 4236, 4449, 4651, 4838, 5006, 5147,
5255, 5322, 5339, 5295, 5180, 4983, 4692, 4293, 3776, 3129, 2341, 1405, 317, -924,
-2314, -3840, -5485, -7223, -9021, -10836, -12622, -14325, -15891, -17264, -18393, -19235
};

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
    //delay(2000);
}

void stopVibration(Haptic_Driver& driver) {
    driver.setVibrate(0);
    vibrating = false;
    Serial.println("[STOP] Vibration stopped");
}

void updateDelayFromTargetHz() {
    float cycleDurationMs = 1000.0 / targetHz;
    delayPerSampleUs_rt = (cycleDurationMs * 1000.0) / waveformSize;
    Serial.print("Updated delayPerSampleUs: ");
    Serial.println(delayPerSampleUs);
}

//====================generate signal==============
void generateNegDatTrial()
{
    for (int i = 0; i < waveformSize; i++) {
        negDatTrial[i] = -dat[i];
    }
    Serial.println("negative generated");
}

//=================setup===========================
void setup() {
    
    Serial.begin(115200);
    //DA7280setup();
    Serial.println("Hello Helllo AND Initiated");

    //titan LF
    analogWriteResolution(12);
    Serial.println("TitanLF Initiated");

}
//=================setup===========================


// the loop function runs over and over again until power down or reset
void loop() 
{
    if (vibrating && millis() - vibStart >= vibDuration)
    {
        stopVibration(hapDrive[0]);
    }

    if (Serial.available() > 0)
    {
        char command = Serial.read();

        if (command == 'A')
        {
            //activate DA7280 
            soleDA7280(hapDrive[0], 0, burst500ms, 100, 175.0);
        }

        else if (command == 'B')
        {
            //activate DA7280 
            //soleDA7280(hapDrive[0], 0, burst500ms, 100, 175.0);
            //delay(100);

            //initiate the targetHz
            updateDelayFromTargetHz();

            //activate titan LF
            for (int repeat = 0; repeat < 10; repeat++)
            {
                for (int i = 0; i < waveformSize; i++)
                {
                    int val = dat[i];
                    int dacValue = map(val, -32767, 32767, 0, 4095);
                    analogWrite(DAC_PIN_A21, dacValue);
                    Serial.println(dacValue);
                    delayMicroseconds((int)delayPerSampleUs_rt);  //40hz
                }
            }
            delay(150);
        }
    }

    
}
