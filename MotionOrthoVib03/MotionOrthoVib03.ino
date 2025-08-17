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
    for (int i = 0; i < 256; i++) high_highPR[i] = -high_high[i];

    //envelope
    makeADSR(0.10f, 0.20f, 0.0f);
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

//================rise and Fall 비율 calculation==========
// 1) 파형에서 피크 인덱스 찾기 (여기선 최대값 기준)
int findPeakIndex(const int16_t* w, int N) {
    int p = 0;
    int16_t mx = w[0];
    for (int i = 1; i < N; ++i) {
        if (w[i] > mx) { mx = w[i]; p = i; }
    }
    return p;
}

// 2) 배열의 일부 구간만 재생 (지연을 인자로)
void playArraySegmentWithGain(const int16_t* w, int i0, int i1_exclusive,
    float gain, int dacPin, uint32_t delayPerSampleUs) {
    for (int i = i0; i < i1_exclusive; ++i) {
        int32_t v = (int32_t)(w[i] * gain);
        // 너의 내부 write 함수로 교체 (예: analogWriteResolution(12) + analogWrite),
        // 또는 playArrayWithGainCentered 내부 코드와 동일하게 출력
        analogWrite(dacPin, (uint16_t)((v + 32768) >> 4)); // 예시: 12-bit로 매핑
        delayMicroseconds(delayPerSampleUs);
    }
}

// 3) 전체를 rise/fall 서로 다른 지연으로 재생
void playAsymTiming(const int16_t* w, int N, float gain, int dacPin,
    uint32_t delayRiseUs, uint32_t delayFallUs) {
    int p = findPeakIndex(w, N);
    // rise: [0 .. p] , fall: [p .. N)
    playArraySegmentWithGain(w, 0, p + 1, gain, dacPin, delayRiseUs);
    playArraySegmentWithGain(w, p + 1, N, gain, dacPin, delayFallUs);
}

// 원하는 rise:fall 비율(rRise:rFall)로 wave_asymHalfSine을 재생
// - 전체 주기(= delayPerSampleUs * N)는 유지
// - rise 구간과 fall 구간에 서로 다른 per-sample 지연을 배분
void playHalfSineWithRatio(const int16_t* w, int N, float gain, int dacPin,
    float delayPerSampleUs, float rRise, float rFall,
    int repeats = 5)
{
    // 전체 주기(us)
    uint32_t T_total_us = (uint32_t)(delayPerSampleUs * N);

    // 비율→비중
    float k = rRise / (rRise + rFall);

    // 피크 위치(최대값 기준)
    int peak = findPeakIndex(w, N);
    int Sr = peak + 1;              // rise 샘플 수
    int Sf = N - (peak + 1);        // fall 샘플 수
    if (Sf <= 0) Sf = 1;

    // 구간별 per-sample 지연(us)
    uint32_t delayRiseUs = (uint32_t)((T_total_us * k) / Sr);
    uint32_t delayFallUs = (uint32_t)((T_total_us * (1.0f - k)) / Sf);

    // 재생
    for (int repeat = 0; repeat < repeats; ++repeat) {
        playAsymTiming(w, N, gain, dacPin, delayRiseUs, delayFallUs);
        delay(500);
        Serial.println("peak_high");
    }

    // 디버그
    Serial.print(F("[RATIO] rise:fall = "));
    Serial.print(rRise); Serial.print(':'); Serial.println(rFall);
    Serial.print(F("  delayRiseUs=")); Serial.print(delayRiseUs);
    Serial.print(F("  delayFallUs=")); Serial.println(delayFallUs);
}

//==============DUO play==========================
// rise와 fall에 서로 다른 per-sample 지연을 주면서
// 두 DAC 핀으로 동시 출력 (A22 정상, A21은 invert 가능)
void playAsymTimingDual(const int16_t* w, int N,
    float gainA22, float gainA21,
    int dacPinA22, int dacPinA21,
    uint32_t delayRiseUs, uint32_t delayFallUs,
    bool invertA21)
{
    int p = findPeakIndex(w, N);

    // RISE: [0 .. p]
    for (int i = 0; i <= p; ++i) {
        int32_t v22 = (int32_t)(w[i] * gainA22);
        int32_t v21 = (int32_t)(((invertA21) ? -w[i] : w[i]) * gainA21);

        // 12-bit DAC 매핑 (0..4095), 중앙 바이어스
        analogWrite(dacPinA22, (uint16_t)((v22 + 32768) >> 4));
        analogWrite(dacPinA21, (uint16_t)((v21 + 32768) >> 4));
        delayMicroseconds(delayRiseUs);
    }

    // FALL: [p+1 .. N-1]
    for (int i = p + 1; i < N; ++i) {
        int32_t v22 = (int32_t)(w[i] * gainA22);
        int32_t v21 = (int32_t)(((invertA21) ? -w[i] : w[i]) * gainA21);

        analogWrite(dacPinA22, (uint16_t)((v22 + 32768) >> 4));
        analogWrite(dacPinA21, (uint16_t)((v21 + 32768) >> 4));
        delayMicroseconds(delayFallUs);
    }
}

// ratio( rRise : rFall )에 맞춰 지연을 배분하고, 듀얼로 동시 재생
void playHalfSineWithRatioDual(const int16_t* w, int N,
    float gainA22, float gainA21,
    int dacPinA22, int dacPinA21,
    float delayPerSampleUs, float rRise, float rFall,
    bool invertA21, int repeats = 5)
{
    // 전체 주기(us)는 유지
    const uint32_t T_total_us = (uint32_t)(delayPerSampleUs * N);
    const float k = rRise / (rRise + rFall);

    const int peak = findPeakIndex(w, N);
    int Sr = peak + 1;
    int Sf = N - (peak + 1);
    if (Sf <= 0) Sf = 1;

    const uint32_t delayRiseUs = (uint32_t)((T_total_us * k) / Sr);
    const uint32_t delayFallUs = (uint32_t)((T_total_us * (1.0f - k)) / Sf);

    for (int rep = 0; rep < repeats; ++rep) {
        playAsymTimingDual(w, N, gainA22, gainA21,
            dacPinA22, dacPinA21,
            delayRiseUs, delayFallUs, invertA21);
        delay(500);
        Serial.println("peak_high");
    }

    // 디버그
    Serial.print(F("[DUAL] ratio ")); Serial.print(rRise); Serial.print(':'); Serial.println(rFall);
    Serial.print(F(" delayRiseUs=")); Serial.print(delayRiseUs);
    Serial.print(F(" delayFallUs=")); Serial.println(delayFallUs);
}

//========================================================

//===================duo 시간 차 재생=====================
// 주어진 비율 rRise:rFall에 맞춰 per-sample dt를 계산해 "샘플 출력 스케줄(μs)"을 만든다.
static void buildTimeScheduleUs(const int16_t* w, int N, float delayPerSampleUs,
    float rRise, float rFall, uint32_t* t_us_out) {
    const uint32_t T_total_us = (uint32_t)(delayPerSampleUs * N);
    const float k = rRise / (rRise + rFall);
    const int peak = findPeakIndex(w, N);
    int Sr = peak + 1;
    int Sf = N - (peak + 1);
    if (Sf <= 0) Sf = 1;

    const double dt_rise = (double)T_total_us * k / (double)Sr;
    const double dt_fall = (double)T_total_us * (1.0 - k) / (double)Sf;

    t_us_out[0] = 0;
    for (int i = 1; i < N; ++i) {
        const double dt = (i <= peak) ? dt_rise : dt_fall;
        t_us_out[i] = (uint32_t)llround((double)t_us_out[i - 1] + dt);
    }
}

// A22는 정상, A21은 폴라리티 반전(옵션)해서 "offsetA21_us"만큼 늦게 시작하여 동시 스케줄링 출력
void playHalfSineWithRatioDualOffset(const int16_t* w, int N, float gainA22, float gainA21,int dacPinA22, int dacPinA21,float delayPerSampleUs, float rRise, float rFall,
    bool invertA21, uint32_t offsetA21_us,int repeats = 5)
{
    // DC 중심화(평균 제거)
    long sum = 0;
    for (int i = 0; i < N; ++i) sum += w[i];
    const float mean = (float)sum / (float)N;

    // 시간 스케줄(공통) 생성
    static uint32_t t_us[256];
    buildTimeScheduleUs(w, N, delayPerSampleUs, rRise, rFall, t_us);

    for (int rep = 0; rep < repeats; ++rep) {
        int i22 = 0, i21 = 0;
        uint32_t next22 = (N > 0) ? t_us[0] : UINT32_MAX;
        uint32_t next21 = (N > 0) ? (offsetA21_us + t_us[0]) : UINT32_MAX;

        const uint32_t t0 = micros();

        while (i22 < N || i21 < N) {
            uint32_t target = UINT32_MAX;
            if (i22 < N && t_us[i22] < target) target = t_us[i22];
            if (i21 < N && (offsetA21_us + t_us[i21]) < target) target = offsetA21_us + t_us[i21];

            // busy-wait until next event
            while ((uint32_t)(micros() - t0) < target) { /* spin */ }

            // 동시 타이밍이면 둘 다 출력
            if (i22 < N && t_us[i22] == target) {
                // A22 출력 (센터 매핑)
                float v = (w[i22] - mean) * gainA22;
                long v16 = lroundf(v);
                v16 = constrain(v16, -32767, 32767);
                analogWrite(dacPinA22, (uint16_t)((v16 + 32768) >> 4));
                i22++;
            }
            if (i21 < N && (offsetA21_us + t_us[i21]) == target) {
                // A21 출력 (폴라리티 반전 옵션 + 센터 매핑)
                int16_t src = invertA21 ? (int16_t)(-w[i21]) : w[i21];
                float v = (src - mean) * gainA21;
                long v16 = lroundf(v);
                v16 = constrain(v16, -32767, 32767);
                analogWrite(dacPinA21, (uint16_t)((v16 + 32768) >> 4));
                i21++;
            }
        }

        delay(500);
        Serial.println(F("peak_high"));
    }

    // 디버그
    Serial.print(F("[DUAL-OFFSET] rRise:rFall="));
    Serial.print(rRise); Serial.print(':'); Serial.println(rFall);
    Serial.print(F("  offsetA21_us=")); Serial.println(offsetA21_us);
}
//===============================================================================

//=================================envelope change===============================
float envA22[256];
// 간단한 ADSR 스타일 (비율 기반): attack(10%), sustain(20%), decay(70%)
void makeADSR(float aRatio = 0.10f, float sRatio = 0.20f, float dEnd = 0.0f) {
    int N = waveformSize;
    int aN = max(1, (int)(N * aRatio));
    int sN = max(0, (int)(N * sRatio));
    int dN = max(1, N - (aN + sN));

    // Attack: 0 -> 1
    for (int i = 0; i < aN; ++i) {
        envA22[i] = (float)i / (float)(aN - 1);
    }
    // Sustain: 1 유지
    for (int i = 0; i < sN; ++i) {
        envA22[aN + i] = 1.0f;
    }
    // Decay: 1 -> dEnd
    for (int i = 0; i < dN; ++i) {
        float t = (float)i / (float)(dN - 1);
        envA22[aN + sN + i] = 1.0f + t * (dEnd - 1.0f);
    }
}

//지수 감쇠
void makeExpDecay(float k = 4.0f) { // k가 클수록 빨리 감소
    for (int i = 0; i < waveformSize; ++i) {
        float x = (float)i / (float)(waveformSize - 1);
        envA22[i] = expf(-k * x);
    }
}

void playHalfSineWithRatioDualOffsetEnv(
    const int16_t* w, int N,
    float gainA22, float gainA21,
    int dacPinA22, int dacPinA21,
    float delayPerSampleUs, float rRise, float rFall,
    bool invertA21, uint32_t offsetA21_us,
    const float* env,           // <= A22에 곱할 envelope(길이 N, 0~1)
    int repeats = 5
) {
    // 평균 제거(센터 매핑용)
    long sum = 0;
    for (int i = 0; i < N; ++i) sum += w[i];
    const float mean = (float)sum / (float)N;

    // 공통 시간 스케줄
    static uint32_t t_us[256];
    buildTimeScheduleUs(w, N, delayPerSampleUs, rRise, rFall, t_us);

    for (int rep = 0; rep < repeats; ++rep) {
        int i22 = 0, i21 = 0;
        const uint32_t t0 = micros();

        while (i22 < N || i21 < N) {
            uint32_t target = UINT32_MAX;
            if (i22 < N && t_us[i22] < target) target = t_us[i22];
            if (i21 < N && (offsetA21_us + t_us[i21]) < target) target = offsetA21_us + t_us[i21];

            while ((uint32_t)(micros() - t0) < target) { /* busy wait */ }

            if (i22 < N && t_us[i22] == target) {
                // A22: envelope 적용
                float shaped = (w[i22] - mean) * gainA22 * (env ? env[i22] : 1.0f);
                long v16 = lroundf(shaped);
                v16 = constrain(v16, -32767, 32767);
                analogWrite(dacPinA22, (uint16_t)((v16 + 32768) >> 4));
                i22++;
            }
            if (i21 < N && (offsetA21_us + t_us[i21]) == target) {
                // A21: 기존과 동일 (envelope 미적용)
                int16_t src = invertA21 ? (int16_t)(-w[i21]) : w[i21];
                float v = (src - mean) * gainA21;
                long v16 = lroundf(v);
                v16 = constrain(v16, -32767, 32767);
                analogWrite(dacPinA21, (uint16_t)((v16 + 32768) >> 4));
                i21++;
            }
        }

        delay(500);
        Serial.println(F("peak_high"));
    }

    Serial.print(F("[DUAL-OFFSET-ENV] rRise:rFall="));
    Serial.print(rRise); Serial.print(':'); Serial.println(rFall);
    Serial.print(F("  offsetA21_us=")); Serial.println(offsetA21_us);
    Serial.println(F("  (A22 envelope applied)"));
}



//===============================================================================


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
            //YY
            // 
            //update hz
            updateDelayFromTargetHz();

            //for comb1 (yz) 
            const float GAIN = 3.0f; 
            for (int repeat = 0; repeat < 10; repeat++)
            {
                //Left flesh
                playArrayWithGainCentered(high_high, waveformSize, GAIN, DAC_PIN_A22, delayPerSampleUs_rt);
                delay(500); //(phase)

                //시간차 조정 어떻게 하냐? 
                
                //Right Flesh
                playArrayWithGainCentered(high_highPR, waveformSize, GAIN, DAC_PIN_A21, delayPerSampleUs_rt);
                //하나의 pulse 이후 쉬기
                delay(500);
            }

        }
        else if (command == '1')
        {
            updateDelayFromTargetHz();
            const float GAIN = 3.0f;
            // rise:fall = 1:2
            playHalfSineWithRatio(wave_asymHalfSine, waveformSize, GAIN, DAC_PIN_A22,
                delayPerSampleUs_rt, 1.0f, 2.0f, /*repeats=*/5);
        }
        else if (command == '2')
        {
            updateDelayFromTargetHz();
            const float GAIN = 3.0f;
            // rise:fall = 1:3
            playHalfSineWithRatio(wave_asymHalfSine, waveformSize, GAIN, DAC_PIN_A22,
                delayPerSampleUs_rt, 1.0f, 3.0f, /*repeats=*/5);
        }
        else if (command == '3')
        {
            updateDelayFromTargetHz();
            const float GAIN = 3.0f;
            // rise:fall = 2:1
            playHalfSineWithRatio(wave_asymHalfSine, waveformSize, GAIN, DAC_PIN_A22,
                delayPerSampleUs_rt, 2.0f, 1.0f, /*repeats=*/5);
        }


        else if (command == '4')
        {
            // 목표 Hz 적용
            updateDelayFromTargetHz();

            const float GAIN_A22 = 3.0f;  // A22(정상)
            const float GAIN_A21 = 3.0f;  // A21(반대 파형)

            // 요구사항: A22 = 1:2, A21 = "반대 파형(폴라리티 반전)"
            const float rRise = 1.0f, rFall = 2.0f;
            const bool invertA21 = true;  // 여기만 true면 됨 (타이밍은 동일, 부호만 반전)

            playHalfSineWithRatioDual(wave_asymHalfSine, waveformSize,
                GAIN_A22, GAIN_A21,
                DAC_PIN_A22, DAC_PIN_A21,
                delayPerSampleUs_rt,
                rRise, rFall,
                invertA21,
                /*repeats=*/5);
        }


        else if (command == '5')
        {
            updateDelayFromTargetHz();

            const float GAIN_A22 = 3.0f;   // A22 (정상)
            const float GAIN_A21 = 3.0f;   // A21 (반대 파형)
            const float rRise = 1.0f, rFall = 2.0f; // 1:2 비율
            const bool  invertA21 = true;  // A21 폴라리티 반전
            const uint32_t OFFSET_A21_US = 50000; // A22가 50 ms 먼저 (A21은 50 ms 지연)

            playHalfSineWithRatioDualOffset(
                wave_asymHalfSine, waveformSize,
                GAIN_A22, GAIN_A21,
                DAC_PIN_A22, DAC_PIN_A21,
                delayPerSampleUs_rt,
                rRise, rFall,
                invertA21, OFFSET_A21_US,
                /*repeats=*/5
            );
        }
        else if (command == '6')
        {
            const float GAIN_A22 = 3.0f;   // A22 (정상)
            const float GAIN_A21 = 3.0f;   // A21 (반대 파형)
            const float rRise = 1.0f, rFall = 2.0f; // 1:2 비율
            const bool  invertA21 = true;  // A21 폴라리티 반전
            const uint32_t OFFSET_A21_US = 30000; // A22가 50 ms 먼저 (A21은 50 ms 지연)

            playHalfSineWithRatioDualOffset(
                wave_asymHalfSine, waveformSize,
                GAIN_A22, GAIN_A21,
                DAC_PIN_A22, DAC_PIN_A21,
                delayPerSampleUs_rt,
                rRise, rFall,
                invertA21, OFFSET_A21_US,
                /*repeats=*/5
            );

        }

        else if (command == '7')
        {
            const float GAIN_A22 = 3.0f;   // A22 (정상)
            const float GAIN_A21 = 5.0f;   // A21 (반대 파형)
            const float rRise = 1.0f, rFall = 2.0f; // 1:2 비율
            const bool  invertA21 = true;  // A21 폴라리티 반전
            const uint32_t OFFSET_A21_US = 50000; // A22가 50 ms 먼저 (A21은 50 ms 지연)

            playHalfSineWithRatioDualOffset(
                wave_asymHalfSine, waveformSize,
                GAIN_A22, GAIN_A21,
                DAC_PIN_A22, DAC_PIN_A21,
                delayPerSampleUs_rt,
                rRise, rFall,
                invertA21, OFFSET_A21_US,
                /*repeats=*/5
            );

        }

        else if (command == '8')
        {
            updateDelayFromTargetHz();

            const float GAIN_A22 = 3.0f;   // A22
            const float GAIN_A21 = 5.0f;   // A21
            const float rRise = 1.0f, rFall = 2.0f;
            const bool  invertA21 = true;
            const uint32_t OFFSET_A21_US = 50000; // 50 ms

            // 필요 시 런타임에도 다른 형태로 바꿔볼 수 있음:
            // makeADSR(0.05f, 0.15f, 0.0f);
            // makeExpDecay(6.0f);

            playHalfSineWithRatioDualOffsetEnv(
                wave_asymHalfSine, waveformSize,
                GAIN_A22, GAIN_A21,
                DAC_PIN_A22, DAC_PIN_A21,
                delayPerSampleUs_rt,
                rRise, rFall,
                invertA21, OFFSET_A21_US,
                envA22,                 // <= A22에만 envelope 적용!
                /*repeats=*/5
            );
        }

        if (command == 'F')
        {
            updateDelayFromTargetHz();
            const float GAIN = 3.0f;
            //slowReturnTwice2
            for (int repeat = 0; repeat < 5; repeat++)
            {
                playArrayWithGainCentered(wave_amBurstAsym, waveformSize, GAIN, DAC_PIN_A22, delayPerSampleUs_rt);
                delay(500);
                Serial.println("peak_high");
            }

        }
        else if (command == 'L') 
        {                 // 왼쪽 킥

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

