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

int loopTimesFive = 5;
int loopTimesThree = 3;

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

    //time phase 
    for (int i = 0; i < 256; i++) high_highTP[i] = high_high[255 - i];

    //Polarity reverse
    for (int i = 0; i < 256; i++) high_highPR[i] = high_high[255 - i];
    
}
//=================setup===========================

//================gain setting=====================
inline int toDac_12bit_centered(int16_t v) {
    // -32767..32767 -> 0..4095
    return map(v, -32767, 32767, 0, 4095);
}

void playArrayWithGainCentered(const int16_t* arr, int n, float gain, int dacPin, float delayUs) {
    // 평균값 산출
    long sum = 0;
    for (int i = 0; i < n; ++i) sum += arr[i];
    float mean = (float)sum / (float)n;

    for (int i = 0; i < n; ++i) {
        float v = (arr[i] - mean) * gain;                // 중심화 + 게인
        int16_t v16 = (int16_t)constrain((long)lround(v), -32767, 32767);
        analogWrite(dacPin, toDac_12bit_centered(v16));
        delayMicroseconds((int)delayUs);
    }
}
//================gain setting=====================

//느린 복귀를 위해 raw 
void playArrayRaw(const int16_t* arr, int n, float gain, int dacPin, float delayUs) {
    for (int i = 0; i < n; i++) {
        long v = lroundf(arr[i] * gain);
        v = constrain(v, -32767, 32767);
        int dac = map((int16_t)v, -32767, 32767, 0, 4095);
        analogWrite(dacPin, dac);
        delayMicroseconds((int)delayUs);
    }
}

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
            startVibration(hapDrive, burst75ms);

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
            delay(200);  // pulse 간 간격
        }
        else if (command == '2')
        {
            updateDelayFromTargetHz();
            const float GAIN = 3.0f; // <- 필요 시 2~6 사이에서 올려보며 조정
            for (int repeat = 0; repeat < 5; repeat++) {
                playArrayWithGainCentered(high_high, waveformSize, GAIN, DAC_PIN_A22, delayPerSampleUs_rt);
                delay(500);
            }
            //한바퀴 끝나고 그다음에 어떻게 할지임 
            delay(150);
            //delay(500); 하면 개빨라지는디
            //delay(1000);
        }

        else if (command == '3')
        {
            updateDelayFromTargetHz();

            // high_high 배열의 시간 역전 
            const float GAIN = 3.0f; // <- 필요 시 2~6 사이에서 올려보며 조정
            for (int repeat = 0; repeat < 5; repeat++) {
                playArrayWithGainCentered(high_highTP, waveformSize, GAIN, DAC_PIN_A22, delayPerSampleUs_rt);
                delay(500);
            }
        }

        //high_highPR : change Polarity 
        else if (command == '4')
        {
            //주기를 느리게~ 하는 
            updateDelayFromTargetHz();

            //high high 배열의 부호 반전 
            for (int i = 0; i < 256; i++) high_high[i] = -high_high[i];

            const float GAIN = 3.0f;
            for (int repeat = 0; repeat < 5; repeat++)
            {
                playArrayWithGainCentered(high_highPR, waveformSize, GAIN, DAC_PIN_A22, delayPerSampleUs_rt);
                delay(500);
                Serial.println("peak_high");
            }
        }

        else if (command == '5')
        {
            //주기를 느리게~ 하는 
            updateDelayFromTargetHz();

            const float GAIN = 3.0f;
            for (int repeat = 0; repeat < 5; repeat++)
            {
                playArrayWithGainCentered(slowUpScale, waveformSize, GAIN, DAC_PIN_A22, delayPerSampleUs_rt);
                delay(500);
                Serial.println("peak_high");
            }
        }
        else if (command == '6')
        {
            updateDelayFromTargetHz();
            const float GAIN = 3.0f;
            //slowReturnTwice2
            for (int repeat = 0; repeat < 5; repeat++)
            {
                playArrayWithGainCentered(slowReturnTwice2, waveformSize, GAIN, DAC_PIN_A22, delayPerSampleUs_rt);
                delay(500);
                Serial.println("peak_high");
            }

        }


        if (command == 'F')
        {
            // 아주 단순: 미드(2048)에서 짧게 위로 스텝, 천천히 복귀
            //DAC를 중간값 2048로 설정하고, 200ms 유지함. 
            //-> 기준점(미드)에서 대기 
            analogWrite(DAC_PIN_A22, 2048); delay(200);

            //갑자기 3400(≈ 2.74V)로 올려 30ms 유지.
            //    → 손에 ‘툭’ 하고 빠른 임팩트(양의 스텝).-- 가장 큰 impact 

            //선형으로 천천히 내려옴 (그래서 잘 안느껴짐)
            analogWrite(DAC_PIN_A22, 3400); delay(30);      // 툭
            for (int k = 0; k < 300; k++) {                        // 300ms 동안 천천히 복귀
                int v = 3400 - (k * (3400 - 2048) / 300);
                analogWrite(DAC_PIN_A22, v);
                delay(1);
            }
            delay(200);

        }
        else if (command == 'L') {                 // 왼쪽 킥
            targetHz = 3.2f;                         // 복귀 ~300ms
            updateDelayFromTargetHz();
            const float GAIN = 3.0f;
            playArrayRaw(leftKickSlowReturn, 256, GAIN, DAC_PIN_A22, delayPerSampleUs_rt);
        }

        else if (command == 'A')
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

