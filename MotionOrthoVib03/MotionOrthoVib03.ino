/*
 Name:		MotionOrthoVib03.ino
 Created:	2025-08-03 오전 11:41:25
 Author:	HCITECH_01
*/

// the setup function runs once when you press reset or power the board
#include "waveforms.h"
#include <Wire.h>
#include "Haptic_Driver.h"
#include <Audio.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>

//self-made h file
#include "waveforms.h"

//================DA7280====================
Haptic_Driver hapDrive;

int burst75ms = 75;
int burst200ms = 200;
int burst500ms = 500;

float currentFreq = 10.0;
bool vibrating = false;
unsigned long vibStart = 0;
unsigned long vibDuration = 0;
//================DA7280====================


//================DA7280 setup====================
void initDA7280(Haptic_Driver& driver) {
    

    if (!driver.begin()) {
        Serial.println("DA7280 not found.");
        return;
    }

    Serial.println("DA7280 initialized.");

    driver.setActuatorType(LRA_TYPE);
    driver.setActuatorLRAfreq(175.0);
    driver.enableFreqTrack(false);
    driver.setOperationMode(DRO_MODE);
}
//================DA7280 setup====================

//==================titanLF setup====================
const int DAC_PIN_A21 = A21; //Teensy 3.5 DAC0 == R in
const int DAC_PIN_A22 = A22; //Teensy 3.5 DAC1 == L in

const int waveformSize = 256;
int waveformIndex = 0;
float targetHz = 40;
float cycleDurationMs = 1000.0 / targetHz;
float delayPerSampleUs = (cycleDurationMs * 1000.0) / waveformSize;
float delayPerSampleUs_rt = (cycleDurationMs * 1000.0) / waveformSize;
int16_t negDat[256];
int16_t negDatTrial[256];

int newWaveCase;

//===========Arrange DA7280=======================
void startVibration(Haptic_Driver& driver, int duration) {
    driver.setVibrate(127);
    vibStart = millis();
    vibDuration = duration;
    vibrating = true;
    Serial.print("[VIBRATE] ");
    Serial.print(duration);
    Serial.println(" ms");
}


void stopVibration(Haptic_Driver& driver) {
    driver.setVibrate(0);
    vibrating = false;
    Serial.println("[STOP] Vibration stopped");
}

void updateDelayFromTargetHz() {
    float cycleDurationMs = 1000.0 / targetHz;
    delayPerSampleUs_rt = (cycleDurationMs * 1000.0) / waveformSize;
    //Serial.print("Updated delayPerSampleUs: ");
    //Serial.println(delayPerSampleUs);
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

    Wire.begin();
    Serial.begin(115200);
    while (!Serial); // USB serial 연결 대기
    delay(100);        // (선택) 약간의 추가 안정화 대기

    
    Serial.println("Hello Helllo AND Initiated");
    initDA7280(hapDrive);

    //titan LF
    analogWriteResolution(12);
    Serial.println("TitanLF Initiated");

    //generate negative
    generateNegDatTrial();

    CreateAllWaveforms();  // << 여기서 한 번에 생성!
    generatePositiveBiasedWaveform();
}
//=================setup===========================


// the loop function runs over and over again until power down or reset
void loop() 
{
    //generate new waveform 


    if (vibrating && millis() - vibStart >= vibDuration)
    {
        stopVibration(hapDrive);
    }

    if (Serial.available() > 0)
    {
        char command = Serial.read();
        if (command == '0')
        {
            //hz update
            updateDelayFromTargetHz();

            for (int repeat = 0; repeat < 2; repeat++) {  // pulse 10번 반복
                for (int i = 0; i < waveformSize; i++) {
                    int val = newWave_0[i];

                    //Teensy에 있는 DAC 는 0~4095 (12비트) 사이 숫자만 출력이 가능하니까 -32767 ~ +32767 값을 0~4095fh 변환해주는거임~ 

                    //previously,맵핑 방향성
                    int dacValue = map(val, -32767, 32767, 0, 4095);

                    //출력
                    analogWrite(DAC_PIN_A22, dacValue);
                    //각각의 점 출력 전에 120마이크로초 기다리는것 --> 샘플링 속도를 조정 
                    //Serial.println(dacValue);
                    delayMicroseconds((int)delayPerSampleUs_rt);  //40hz
                }
            }
            delay(150);  // pulse 간 간격
        }
        else if (command == '1')
        {
            //hz update
            updateDelayFromTargetHz();

            for (int repeat = 0; repeat < 2; repeat++) {  // pulse 10번 반복
                for (int i = 0; i < waveformSize; i++) {
                    int val = newWave_1[i];

                    //Teensy에 있는 DAC 는 0~4095 (12비트) 사이 숫자만 출력이 가능하니까 -32767 ~ +32767 값을 0~4095fh 변환해주는거임~ 

                    //previously,맵핑 방향성
                    int dacValue = map(val, -32767, 32767, 0, 4095);

                    //출력
                    analogWrite(DAC_PIN_A22, dacValue);
                    //각각의 점 출력 전에 120마이크로초 기다리는것 --> 샘플링 속도를 조정 
                    //Serial.println(dacValue);
                    delayMicroseconds((int)delayPerSampleUs_rt);  //40hz
                }
            }
            delay(150);  // pulse 간 간격
        }
        else if (command == '2')
        {
            //hz update
            updateDelayFromTargetHz();

            for (int repeat = 0; repeat < 2; repeat++) {  // pulse 10번 반복
                for (int i = 0; i < waveformSize; i++) {
                    int val = newWave_2[i];

                    //Teensy에 있는 DAC 는 0~4095 (12비트) 사이 숫자만 출력이 가능하니까 -32767 ~ +32767 값을 0~4095fh 변환해주는거임~ 

                    //previously,맵핑 방향성
                    int dacValue = map(val, -32767, 32767, 0, 4095);

                    //출력
                    analogWrite(DAC_PIN_A22, dacValue);
                    //각각의 점 출력 전에 120마이크로초 기다리는것 --> 샘플링 속도를 조정 
                    //Serial.println(dacValue);
                    delayMicroseconds((int)delayPerSampleUs_rt);  //40hz
                }
            }
            delay(150);  // pulse 간 간격
        }
        else if (command == '3')
        {
            //hz update
            updateDelayFromTargetHz();

            for (int repeat = 0; repeat < 2; repeat++) {  // pulse 10번 반복
                for (int i = 0; i < waveformSize; i++) {
                    int val = newWave_3[i];

                    //Teensy에 있는 DAC 는 0~4095 (12비트) 사이 숫자만 출력이 가능하니까 -32767 ~ +32767 값을 0~4095fh 변환해주는거임~ 

                    //previously,맵핑 방향성
                    int dacValue = map(val, -32767, 32767, 0, 4095);

                    //출력
                    analogWrite(DAC_PIN_A22, dacValue);
                    //각각의 점 출력 전에 120마이크로초 기다리는것 --> 샘플링 속도를 조정 
                    //Serial.println(dacValue);
                    delayMicroseconds((int)delayPerSampleUs_rt);  //40hz
                }
            }
            delay(150);  // pulse 간 간격
        }
        else if (command == '4')
        {
            //hz update
            updateDelayFromTargetHz();

            for (int repeat = 0; repeat < 2; repeat++) {  // pulse 10번 반복
                for (int i = 0; i < waveformSize; i++) {
                    int val = newWave_4[i];

                    //Teensy에 있는 DAC 는 0~4095 (12비트) 사이 숫자만 출력이 가능하니까 -32767 ~ +32767 값을 0~4095fh 변환해주는거임~ 

                    //previously,맵핑 방향성
                    int dacValue = map(val, -32767, 32767, 0, 4095);

                    //출력
                    analogWrite(DAC_PIN_A22, dacValue);
                    //각각의 점 출력 전에 120마이크로초 기다리는것 --> 샘플링 속도를 조정 
                    //Serial.println(dacValue);
                    delayMicroseconds((int)delayPerSampleUs_rt);  //40hz
                }
            }
            delay(150);  // pulse 간 간격
        }

        if (command == 'F')
        {
            //single LRA 
            startVibration(hapDrive, burst75ms);
        }
        if (command == 'A')
        {
            //hz update
            updateDelayFromTargetHz();

            for (int repeat = 0; repeat < 2; repeat++) {  // pulse 10번 반복
                for (int i = 0; i < waveformSize; i++) {
                    int val = dat[i];

                    //Teensy에 있는 DAC 는 0~4095 (12비트) 사이 숫자만 출력이 가능하니까 -32767 ~ +32767 값을 0~4095fh 변환해주는거임~ 

                    //previously,맵핑 방향성
                    int dacValue = map(val, -32767, 32767, 0, 4095);

                    //출력
                    analogWrite(DAC_PIN_A22, dacValue);
                    //각각의 점 출력 전에 120마이크로초 기다리는것 --> 샘플링 속도를 조정 
                    //Serial.println(dacValue);
                    delayMicroseconds((int)delayPerSampleUs_rt);  //40hz
                }
            }
            delay(150);  // pulse 간 간격
        }


        else if (command == 'B')
        {
            Serial.println("B is pressed: dat2 반대 방향으로 출력 (negDatTrial)");
            //hz update
            updateDelayFromTargetHz();

            for (int repeat = 0; repeat < 2; repeat++) {
                for (int i = 0; i < waveformSize; i++) {
                    int val = negDatTrial[i];
                    int dacValue = map(val, -32767, 32767, 0, 4095);

                    analogWrite(DAC_PIN_A21, dacValue);
                    //Serial.println(dacValue);
                    delayMicroseconds((int)delayPerSampleUs_rt);
                }
            }
            delay(150);
        }

        else if (command == 'C')
        {
            //activate DA7280 
            startVibration(hapDrive, burst75ms);
            //delay(100);

            //initiate the targetHz
            updateDelayFromTargetHz();

            //activate titan LF
            for (int repeat = 0; repeat < 5; repeat++)
            {
                for (int i = 0; i < waveformSize; i++)
                {
                    int val = dat[i];

                    //이걸로 intensity를 결정하는 거임. 
                    //int val_scaled = constrain((int)(val * gain), -32767, 32767);
                    int dacValue = map(val, -32767, 32767, 0, 4095);
                    //Serial.println(dacValue);

                    analogWrite(DAC_PIN_A21, dacValue);
                    //analogWrite(DAC_PIN_A22, dacValue);

                    delayMicroseconds((int)delayPerSampleUs_rt);  //40hz

                }
            }
            delay(150);
        }
        else if (command == 'D')
        {
            //initiate the targetHz
            updateDelayFromTargetHz();

            //activate titan LF
            for (int repeat = 0; repeat < 5; repeat++)
            {
                for (int i = 0; i < waveformSize; i++)
                {
                    int val = dat[i];

                    //이걸로 intensity를 결정하는 거임. 
                    //int val_scaled = constrain((int)(val * gain), -32767, 32767);
                    int dacValue = map(val, -32767, 32767, 0, 4095);
                    //Serial.println(dacValue);

                    analogWrite(DAC_PIN_A21, dacValue);
                    analogWrite(DAC_PIN_A22, dacValue);

                    delayMicroseconds((int)delayPerSampleUs_rt);  //40hz

                }
            }
            delay(150);
        }
        else if (command == 'G')
        {
            //initiate the targetHz
            updateDelayFromTargetHz();

            //activate titan LF
            for (int repeat = 0; repeat < 5; repeat++)
            {
                for (int i = 0; i < waveformSize; i++)
                {
                    int val = dat[i];

                    //이걸로 intensity를 결정하는 거임. 
                    //int val_scaled = constrain((int)(val * gain), -32767, 32767);
                    int dacValue = map(val, -32767, 32767, 0, 4095);
                    //Serial.println(dacValue);

                    analogWrite(DAC_PIN_A22, dacValue);
                    //analogWrite(DAC_PIN_A22, dacValue);

                    delayMicroseconds((int)delayPerSampleUs_rt);  //40hz

                }
            }
            delay(150);
        }
        else if (command == 'H')
        {
            //initiate the targetHz
            updateDelayFromTargetHz();

            //activate titan LF
            for (int repeat = 0; repeat < 5; repeat++)
            {
                for (int i = 0; i < waveformSize; i++)
                {
                    int val = biasWaveform[i];

                    //이걸로 intensity를 결정하는 거임. 
                    //int val_scaled = constrain((int)(val * gain), -32767, 32767);
                    int dacValue = map(val, -32767, 32767, 0, 4095);
                    //Serial.println(dacValue);

                    analogWrite(DAC_PIN_A22, dacValue);
                    //analogWrite(DAC_PIN_A22, dacValue);

                    delayMicroseconds((int)delayPerSampleUs_rt);  //40hz

                }
            }
            delay(150);
        }
        else if (command == 'I')
        {
            //initiate the targetHz
            updateDelayFromTargetHz();

            //activate titan LF
            for (int repeat = 0; repeat < 5; repeat++)
            {
                for (int i = 0; i < waveformSize; i++)
                {
                    int val = asyTriangular[i];

                    //이걸로 intensity를 결정하는 거임. 
                    //int val_scaled = constrain((int)(val * gain), -32767, 32767);
                    int dacValue = map(val, -32767, 32767, 0, 4095);
                    //Serial.println(dacValue);

                    analogWrite(DAC_PIN_A22, dacValue);
                    //analogWrite(DAC_PIN_A22, dacValue);

                    delayMicroseconds((int)delayPerSampleUs_rt);  //40hz

                }
            }
            delay(150);
        }

    }

    
}

