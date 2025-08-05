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

int16_t newWave[256];  // switch문 바깥에서 선언 필요

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
int16_t datCentered[170] = {
  -10399, -9882, -9039, -7961, -6538, -4972, -2869, -1483, 332, 2130,
  3868, 5413, 7039, 8429, 9669, 10757, 11693, 12481, 13128, 13645,
  14044, 14335, 14532, 14647, 14691, 14674, 14607, 14499, 14358, 14190,
  14003, 13799, 13586, 13368, 13146, 12924, 12705, 12489, 12279, 12075,
  11878, 11689, 11509, 11337, 11175, 11021, 10877, 10741, 10615, 10496,
  10386, 10285, 10191, 10104, 10025, 9952, 9887, 9827, 9774, 9726,
  9684, 9646, 9614, 9586, 9563, 9543, 9528, 9516, 9507, 9502,
  9499, 9499, 9502, 9507, 9514, 9524, 9535, 9548, 9562, 9578,
  9596, 9614, 9633, 9654, 9675, 9697, 9719, 9742, 9765, 9789,
  9813, 9837, 9862, 9886, 9910, 9934, 9958, 9981, 10004, 10027,
  10049, 10070, 10091, 10111, 10130, 10149, 10167, 10185, 10201, 10217,
  10232, 10247, 10260, 10273, 10284, 10295, 10305, 10314, 10322, 10330,
  10336, 10341, 10345, 10349, 10351, 10353, 10353, 10353, 10351, 10349,
  10345, 10341, 10336, 10330, 10322, 10314, 10305, 10295, 10284, 10273,
  10260, 10247, 10232, 10217, 10201, 10185, 10167, 10149, 10130, 10111,
  10091, 10070, 10049, 10027, 10004, 9981, 9958, 9934, 9910, 9886,
  9862, 9837, 9813, 9789, 9765, 9742, 9719, 9697, 9675, 9654
};
int16_t dat2[waveformSize] = {
  0, 822, 1657, 2496, 3331, 4160, 4983, 5800, 6609, 7412, 8207, 8993, 9771,
  10540, 11300, 12050, 12789, 13518, 14236, 14941, 15635, 16316, 16983,
  17637, 18276, 18901, 19510, 20104, 20681, 21242, 21785, 22310, 22818,
  23307, 23777, 24228, 24659, 25070, 25461, 25831, 26181, 26509, 26816,
  27101, 27365, 27606, 27824, 28020, 28194, 28344, 28472, 28576, 28657,
  28714, 28748, 28758, 28743, 28705, 28643, 28556, 28445, 28309, 28148,
  27963, 27752, 27516, 27255, 26968, 26655, 26316, 25952, 25561, 25144,
  24701, 24232, 23736, 23213, 22665, 22089, 21487, 20858, 20203, 19521,
  18813, 18078, 17317, 16530, 15716, 14876, 14010, 13118, 12199, 11255,
  10285, 9289, 8278, 7242, 6181, 5096, 3986, 2853, 1695, 515, -688, -1913,
  -3159, -4427, -5715, -7024, -8353, -9701, -11067, -12452, -13854, -15274,
  -16710, -18162, -19629, -21110, -22605, -24113, -25633, -27164, -28706,
  -30258, -31819, -32767, -32527, -32269, -31994, -31700, -31388, -31058,
  -30710, -30344, -29961, -29559, -29140, -28703, -28249, -27777, -27288,
  -26782, -26258, -25718, -25162, -24588, -23999, -23393, -22771, -22133,
  -21480, -20811, -20126, -19426, -18712, -17983, -17239, -16481, -15709,
  -14923, -14123, -13310, -12484, -11644, -10792, -9938, -9071, -8192,
  -7302, -6400, -5487, -4563, -3629, -2683, -1728, -764, 209, 1191, 2172,
  3151, 4128, 5102, 6074, 7042, 8006, 8967, 9922, 10873, 11818, 12757,
  13689, 14615, 15533, 16443, 17344, 18237, 19120, 19993, 20856, 21709,
  22550, 23380, 24198, 25004, 25796, 26576, 27342, 28094, 28831, 29553,
  30260, 30951, 31626, 32285, 32767
};

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

    //generate new waveform 
    CreateWaveForm(0);

}
//=================setup===========================


// the loop function runs over and over again until power down or reset
void loop() 
{
    if (vibrating && millis() - vibStart >= vibDuration)
    {
        stopVibration(hapDrive);
    }

    if (Serial.available() > 0)
    {
        char command = Serial.read();

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
            //C 비대칭 pulse (the positivie area) 
            int pulseLength = 170;
            updateDelayFromTargetHz();  // delayPerSampleUs_rt 업데이트

            const int datMin = -19754;
            const int datMax = 1049;

            for (int i = 0; i < pulseLength; i++)
            {
                int val = dat[i];  // 원본 진폭
                int dacValue = map(val, datMin, datMax, 0, 4095);  // dat의 실제 범위 기반 정규화
                analogWrite(DAC_PIN_A21, dacValue);
                delayMicroseconds((int)delayPerSampleUs_rt);  // 샘플 간 시간 간격 (40Hz 기준)
            }
            // pulse 끝나고 0V로 안정화
            analogWrite(DAC_PIN_A21, 2048);  // 12bit DAC에서 중심값

        }


    }

    
}

void CreateWaveForm(int selection)
{
    switch (selection) 
    {
        case 0 : //대칭 Sine Wave
            for (int i = 0; i < 256; i++) {
                newWave[i] = (int16_t)(32767 * sin(2 * PI * i / 256));
            }
            break;
        case 1 : //Gaussian-shaped Pulse (비대칭 또는 중심 집중형)
            float sigma = 40.0;  // 폭 조절
            for (int i = 0; i < 256; i++) {
                float x = (i - 128.0);
                newWave[i] = (int16_t)(32767 * exp(-(x * x) / (2 * sigma * sigma)));
            }
            break;
        case 2 : //Half Sine Pulse(0 → sin → 0)
            for (int i = 0; i < 256; i++) {
                newWave[i] = (int16_t)(32767 * sin(PI * i / 256)); // Half sine
            }
            break;
        case 3 : //Trapezoidal Pulse (선형 상승/하강 + plateau)
            int rise = 64, hold = 128, fall = 64;
            for (int i = 0; i < 256; i++) {
                if (i < rise)
                    newWave[i] = (int16_t)(32767 * i / (float)rise);
                else if (i < rise + hold)
                    newWave[i] = 32767;
                else
                    newWave[i] = (int16_t)(32767 * (255 - i) / (float)fall);
            }
            break;
        case 4: //Exponential Decay Pulse
            for (int i = 0; i < 256; i++) {
                newWave[i] = (int16_t)(32767 * exp(-0.03 * i));
            }
            break;
        case 5: // Impact + Residual Vibration
            for (int i = 0; i < 10; i++) {
                newWave[i] = (int16_t)(32767.0 * i / 10.0);  // 급격한 상승
            }
            for (int i = 10; i < 256; i++) {
                float decay = exp(-0.03 * (i - 10));
                float sine = sin(2.0 * PI * (i - 10) / 15.0);
                newWave[i] = (int16_t)(32767.0 * decay * sine);
            }
            break;
        case 6: // Fast Double Pulse
        {
            int pulse_width = 10;    // 각 펄스의 너비
            int pulse_gap = 30;      // 두 펄스 사이 간격

            // 전체 배열 0으로 초기화
            memset(dat, 0, sizeof(dat));

            // 첫 번째 펄스
            for (int i = 0; i < pulse_width; i++) {
                dat[i] = (int16_t)(32767.0 * sin(PI * i / pulse_width));
            }

            // 두 번째 펄스
            for (int i = 0; i < pulse_width; i++) {
                int idx = pulse_gap + i;
                if (idx < 256) {
                    dat[idx] = (int16_t)(32767.0 * sin(PI * i / pulse_width));
                }
            }
        }
        break;

        default:
            Serial.println("Invalid selection");
            break;



    }
}
