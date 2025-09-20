/*
 Name:		MotionOrthoVib03.ino
 Created:	2025-08-03 오전 11:41:25
 Author:	HCITECH_01
*/

// the setup function runs once when you press reset or power the board
#include "MPUsetting.h"
#include "waveforms.h"
#include <Wire.h>
#include "Haptic_Driver.h"
#include <Audio.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <Arduino.h>
#include <Wire.h>

//self-made h file
#include "waveforms.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//============movement 연동=================
// ---- UI event helpers (for HandMotionSpeed.html) ----
static inline void UI_START(char c) { Serial.print(F("[UI] START ")); Serial.println(c); }
static inline void UI_END(char c) { Serial.print(F("[UI] END "));   Serial.println(c); }
//static inline void UI_PEAK() { Serial.println(F("[UI] PEAK")); }

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

//==================actuator setup====================
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
static int16_t wPhase0[256], wPhase180[256];

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
    //analogWriteResolution(12);
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
    //makeADSR(0.10f, 0.20f, 0.0f);

    //polarity reverse for Exploratory study
    for (int i = 0; i < 256; i++)
    {
        slowUpHardDropPR[i] = -wave_slowUpHardDrop[i];
        xpRiseLinFallPR[i] = -wave_expRiseLinFall[i];
        impulseDampedTailPR[i] = -wave_impulseDampedTail[i];
        asymHalfSinePR[i] = -wave_asymHalfSine[i];
        amBurstAsymPR[i] = -wave_amBurstAsym[i];
        quadPushLinReturnPR[i] = -wave_quadPushLinReturn[i];
        impulseDynamicPR[i] = -impulse_dynamic[i];
        impulse_releasePR[i] = -impulse_release[i];
        //Serial.print(impulseDynamicPR[i]);
    }

    
    //Teensy DAC 12bit 이고, 해상도 지정을 해서 내부 스케일이 맞게끔 진행 
    //이거 쓰니까 확실히 다름

    analogWriteResolution(12);

    // setup() 마지막쯤
    if (!mpu9250_beginAccel(ACCEL_FS_8G, /*dlpf=*/2, /*odr_hz=*/1000)) {
        Serial.println("[MPU9250] init failed");
    }
    else {
        Serial.println("[MPU9250] ready");
    }

    // setup() 끝에 한 번 만들어두고…
    makeCompositeF_2F_Phase(wPhase0, 256, 1.0f, 1.0f, 0.0f, true, true);   // 오른쪽 당김 쪽
    makeCompositeF_2F_Phase(wPhase180, 256, 1.0f, 1.0f, -180.0f, true, true);  // 왼쪽 당김 쪽

}
//=================setup===========================

//================gain setting=====================
inline int toDac_12bit_centered(int16_t v) {
    // -32767..32767 -> 0..4095
    return map(v, -32767, 32767, 0, 4095);
}

// 256샘플짜리 파형을 'sec' 동안 원하는 Hz로 반복 재생
void playWaveForSeconds(const int16_t* w, int N, float gain, int dacPin,
    float hz, float sec) {
    // 1사이클 시간(us)
    const float T_us = (1000000.0f / hz);
    // 샘플 간 지연(us)
    const float delayPerSampleUs = T_us / N;

    // 몇 사이클 재생할지
    const uint32_t cycles = (uint32_t)lround(hz * sec);
    for (uint32_t c = 0; c < cycles; ++c) {
        long sum = 0; for (int i = 0; i < N; ++i) sum += w[i];
        const float mean = (float)sum / (float)N;

        // 누적 시간 스케줄(소수 손실 방지)
        uint32_t t0 = micros();
        for (int i = 0; i < N; ++i) {
            float v = (w[i] - mean) * gain;
            long v16 = lroundf(v);
            v16 = constrain(v16, -32767, 32767);
            analogWrite(dacPin, (uint16_t)((v16 + 32768) >> 4));

            // 목표 시각까지 바쁘게 대기(누적 방식으로 정확도↑)
            const uint32_t target = (uint32_t)lround((i + 1) * delayPerSampleUs);
            while ((uint32_t)(micros() - t0) < target) { /* spin */ }
        }
    }
}

// 256샘플짜리 파형(w)을 hz로 sec 동안 반복 재생 (DC 제거 + GAIN 적용 + 정확 타이밍)
void playWaveForSeconds2(const int16_t* w, int N,
    float gain, int dacPin,
    float hz, float sec)
{
    if (hz <= 0.0f || sec <= 0.0f || N <= 0) return;

    // 1사이클(=N샘플) 시간(µs)과 샘플 간 간격(µs)
    const double T_us = 1000000.0 / (double)hz;
    const double dt = T_us / (double)N;

    // 총 재생할 사이클 수 (예: 10 Hz × 1 s = 10 사이클)
    const uint32_t cycles = (uint32_t)llround(hz * sec);
    if (cycles == 0) return;

    // DC 제거(평균값) 계산
    long sum = 0;
    for (int i = 0; i < N; ++i) sum += w[i];
    const float mean = (float)sum / (float)N;

    // 타이밍 기준 시각
    const uint32_t t0 = micros();

    // 총 샘플 수 = N * cycles
    const uint32_t totalSamples = (uint32_t)N * cycles;

    // 누적 목표 시각(µs): 소수 손실 방지를 위해 누적 방식 사용
    double acc_us = 0.0;

    for (uint32_t k = 0; k < totalSamples; ++k) {
        const int i = (int)(k % (uint32_t)N);

        // DC 제거 + GAIN 적용
        float v = (w[i] - mean) * gain;
        long  v16 = lroundf(v);
        v16 = constrain(v16, -32767, 32767);

        // 12-bit DAC 센터 매핑(0..4095), 네 함수 재사용
        analogWrite(dacPin, toDac_12bit_centered((int16_t)v16));

        // 다음 샘플 목표 시각까지 대기 (micros 기반 busy-wait)
        acc_us += dt;
        const uint32_t target = (uint32_t)llround(acc_us);
        while ((uint32_t)(micros() - t0) < target) { /* spin */ }
    }
}


void playArrayWithGainCentered(const int16_t* arr, int n, float gain, int dacPin, float delayUs) {
    // 평균값 산출 (== dc 오프셋) 
    long sum = 0;
    for (int i = 0; i < n; ++i) sum += arr[i];
    float mean = (float)sum / (float)n;

    for (int i = 0; i < n; ++i) {
        float v = (arr[i] - mean) * gain;                // dc 제거 -> 평균화 
        int16_t v16 = (int16_t)constrain((long)lround(v), -32767, 32767);  //clipping 
        //int dacValue = map(val, -32767, 32767, 0, 4095);
        analogWrite(dacPin, toDac_12bit_centered(v16));
        delayMicroseconds((int)delayUs); //샘플간 간격 유지~ 
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
void playAsymTimingDual(const int16_t* w, int N,float gainA22, float gainA21,int dacPinA22, int dacPinA21,uint32_t delayRiseUs, uint32_t delayFallUs,
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

//===========================Polarity Reverse가 포함된 함수======================
void playHalfSineWithRatioDualAltPolarity(const int16_t* w, int N, float gainA22_base, float gainA21_base, int dacPinA22, int dacPinA21,
    float delayPerSampleUs, float rRise, float rFall, bool invertA21_base, int pairRepeats)
    // 쌍 반복 횟수 (정상→반전 = 1쌍))
{
    const uint32_t T_total_us = (uint32_t)(delayPerSampleUs * N);
    const float k = rRise / (rRise + rFall);
    const int peak = findPeakIndex(w, N);
    int Sr = peak + 1;
    int Sf = N - (peak + 1);
    if (Sf <= 0) Sf = 1;

    const uint32_t delayRiseUs = (uint32_t)((T_total_us * k) / Sr);
    const uint32_t delayFallUs = (uint32_t)((T_total_us * (1.0f - k)) / Sf);

    for (int pair = 0; pair < pairRepeats; ++pair) {
        // (1) 정상 극성
        {
            const float g22 = +gainA22_base;
            const float g21 = +gainA21_base;
            playAsymTimingDual(
                w, N,
                g22, g21,
                dacPinA22, dacPinA21,
                delayRiseUs, delayFallUs,
                invertA21_base // A21은 요청대로 반대 위상 유지
            );
            delay(500);
            Serial.println(F("pair_step: normal"));
        }
        // (2) 둘 다 부호 반전 (채널 각각의 -신호)
        {
            const float g22 = -gainA22_base; // A22 반전
            const float g21 = -gainA21_base; // A21 반전
            playAsymTimingDual(
                w, N,
                g22, g21,
                dacPinA22, dacPinA21,
                delayRiseUs, delayFallUs,
                invertA21_base // 관계 유지: A21은 항상 A22와 반대로
            );
            delay(500);
            Serial.println(F("pair_step: inverted"));
        }
    }

    Serial.print(F("[DUAL-ALT] pairs=")); Serial.println(pairRepeats);
    Serial.print(F("  delayRiseUs=")); Serial.print(delayRiseUs);
    Serial.print(F("  delayFallUs=")); Serial.println(delayFallUs);
}

// === (A) 한 사이클을 sec 동안 느리게 재생 ===
// 256샘플짜리 파형 w를 'sec' 동안 단 1사이클만 재생 (DC제거 + GAIN + 정확 타이밍)
void playWaveOneCycleForSeconds(const int16_t* w, int N,
    float gain, int dacPin,
    float sec)
{
    if (sec <= 0.0f || N <= 0) return;

    // 평균값(D C) 제거
    long sum = 0;
    for (int i = 0; i < N; ++i) sum += w[i];
    const float mean = (float)sum / (float)N;

    // 전체 길이를 sec로 고정 → per-sample 간격
    const double T_us = (double)sec * 1e6;       // 전체 주기(한 사이클) us
    const double dt = T_us / (double)N;        // 샘플 간 간격 us (부동소수 누적)

    const uint32_t t0 = micros();
    double acc_us = 0.0;

    for (int i = 0; i < N; ++i) {
        float v = (w[i] - mean) * gain;
        long  v16 = lroundf(v);
        v16 = constrain(v16, -32767, 32767);
        analogWrite(dacPin, toDac_12bit_centered((int16_t)v16));

        acc_us += dt;
        const uint32_t target = (uint32_t)llround(acc_us);
        while ((uint32_t)(micros() - t0) < target) { /* busy-wait */ }
    }
}

// === (A-라이트) 기존 함수를 그대로 활용하는 래퍼 ===
// 내부적으로 hz=1/sec로 호출하여 cycles=1이 되도록 보장
void playWaveOneCycleForSeconds_viaExisting(const int16_t* w, int N,
    float gain, int dacPin,
    float sec)
{
    if (sec <= 0.0f) return;
    const float hz = 1.0f / sec;  // 한 사이클이 sec
    // 혹시 반올림으로 0이 나오는 걸 막기 위해 내부에서 최소 1사이클 보장하도록 수정해도 좋음
    playWaveForSeconds2(w, N, gain, dacPin, hz, sec);
}

// === (B) 부드러운 슬로우 재생(업샘플 + 선형보간) ===
// upsample >= 1 (기본 1). upsample이 크면 더 부드러우나 CPU부담↑.
// 전체 길이는 그대로 'sec'.
void playWaveOneCycleForSeconds_Interpolated(const int16_t* w, int N,
    float gain, int dacPin,
    float sec, int upsample = 4)
{
    if (sec <= 0.0f || N <= 1) return;
    if (upsample < 1) upsample = 1;

    // DC 제거
    long sum = 0;
    for (int i = 0; i < N; ++i) sum += w[i];
    const float mean = (float)sum / (float)N;

    const int M = (N - 1) * upsample + 1;        // 보간 후 총 출력 스텝 수
    const double T_us = (double)sec * 1e6;
    const double dt = T_us / (double)M;

    const uint32_t t0 = micros();
    double acc_us = 0.0;

    for (int step = 0; step < M; ++step) {
        // 보간 인덱스 계산: 0..N-1 범위
        const double pos = (double)step / (double)upsample; // 0..(N-1)
        int i0 = (int)floor(pos);
        int i1 = (i0 + 1 < N) ? (i0 + 1) : i0;
        const float t = (float)(pos - (double)i0);          // 0..1

        // 선형보간
        const float s0 = (w[i0] - mean);
        const float s1 = (w[i1] - mean);
        const float s = s0 + (s1 - s0) * t;

        long v16 = lroundf(s * gain);
        v16 = constrain(v16, -32767, 32767);
        analogWrite(dacPin, toDac_12bit_centered((int16_t)v16));

        acc_us += dt;
        const uint32_t target = (uint32_t)llround(acc_us);
        while ((uint32_t)(micros() - t0) < target) { /* busy-wait */ }
    }
}

/**
 * 한 사이클만 재생 (hz만 지정)
 * - wave w[0..N-1]를 정확히 1사이클 출력
 * - DC 제거 + gain, micros()로 per-sample 타이밍 제어
 */
void playArrayWithGainCentered_1cycle(
    const int16_t* w, int N,
    float gain, int dacPin,
    float hz,
    uint32_t* clipped_out = nullptr
) {
    if (hz <= 0.0f || N <= 0) return;

    const double T_us = 1000000.0 / (double)hz;
    const double dt = T_us / (double)N;

    long sum = 0; for (int i = 0; i < N; ++i) sum += w[i];
    const float mean = (float)sum / (float)N;

    uint32_t clipped = 0;
    const uint32_t t0 = micros();
    double acc_us = 0.0;

    for (int i = 0; i < N; ++i) {
        float v = (w[i] - mean) * gain;
        long  v16 = lroundf(v);
        if (v16 > 32767) { v16 = 32767; ++clipped; }
        else if (v16 < -32767) { v16 = -32767; ++clipped; }

        analogWrite(dacPin, toDac_12bit_centered((int16_t)v16));

        acc_us += dt;
        const uint32_t target = (uint32_t)llround(acc_us);
        while ((uint32_t)(micros() - t0) < target) { /* spin */ }
    }
    if (clipped_out) *clipped_out = clipped;
}

/**
 * N사이클 재생 (시간이 아닌 cycles로 지정)
 * - 총 재생시간을 직접 넣지 않고, 원하는 사이클 수만큼 정확히 출력
 */

void playArrayWithGainCentered_cycles(const int16_t* w, int N, float gain, int dacPin,float hz, uint32_t cycles,
uint32_t* clipped_out = nullptr) {
    if (hz <= 0.0f || N <= 0 || cycles == 0) return;

    const double T_us = 1000000.0 / (double)hz;
    const double dt = T_us / (double)N;

    long sum = 0; for (int i = 0; i < N; ++i) sum += w[i];
    const float mean = (float)sum / (float)N;

    uint32_t clipped = 0;
    const uint32_t t0 = micros();
    const uint32_t totalSamples = (uint32_t)N * cycles;
    double acc_us = 0.0;

    for (uint32_t k = 0; k < totalSamples; ++k) {
        const int i = (int)(k % (uint32_t)N);

        float v = (w[i] - mean) * gain;
        long  v16 = lroundf(v);
        if (v16 > 32767) { v16 = 32767; ++clipped; }
        else if (v16 < -32767) { v16 = -32767; ++clipped; }

        analogWrite(dacPin, toDac_12bit_centered((int16_t)v16));

        acc_us += dt;
        const uint32_t target = (uint32_t)llround(acc_us);
        while ((uint32_t)(micros() - t0) < target) { /* spin */ }
    }
    if (clipped_out) *clipped_out = clipped;
}

//=====================================================asymmetric vibration generator=====================================================
//목적:기본파 + 2차 고조파를 위상차로 합성한 256샘플 파형을 만들어본다. 이미 가지고 있는 함수를 써서 gain 을 맞춰서 플레이갈긴다. 

// 유틸: 배열 평균 제거(DC 제거)
static inline void removeDC_int16(int16_t* w, int N) {
    long sum = 0;
    for (int i = 0; i < N; ++i) sum += w[i];
    const float mean = (float)sum / (float)N;
    for (int i = 0; i < N; ++i) {
        float v = (float)w[i] - mean;
        long  v16 = lroundf(v);
        v16 = constrain(v16, -32767, 32767);
        w[i] = (int16_t)v16;
    }
}

// 유틸: 피크 기준 정규화(최대 절대값을 32767로 맞춤)
static inline void normalizePeak_int16(int16_t* w, int N) {
    int16_t maxAbs = 1;
    for (int i = 0; i < N; ++i) {
        int16_t a = w[i] >= 0 ? w[i] : (int16_t)(-w[i]);
        if (a > maxAbs) maxAbs = a;
    }
    if (maxAbs <= 0) return;
    const float s = 32767.0f / (float)maxAbs;
    for (int i = 0; i < N; ++i) {
        long v = lroundf((float)w[i] * s);
        v = constrain(v, -32767, 32767);
        w[i] = (int16_t)v;
    }
}

/*
기본파 + 2차 고조파 합성 만들기 (한 주기 = N 샘플) 
A1_rel, A2_rel : 상대 진폭 (무단위), 실제 강도는 재생 시, gain 으로 조절 
phi_deg : 2차 성분의 위상 오프셋 (도 단위 : 0, -90, -180) 
removeDC : 평균값 제거 
normalizePeak : 피크 기준으로 쁠마 32767 맞춰 정규화 할지 말지 
*/
static void makeCompositeF_2F_Phase(
    int16_t* out, int N,
    float A1_rel, float A2_rel, float phi_deg,
    bool removeDC = true,
    bool normalizePeak = true
) {
    const float phi = phi_deg * (float)M_PI / 180.0f;
    const float twoPi = 2.0f * (float)M_PI;

    for (int i = 0; i < N; ++i) {
        // 기본파 각도(0..2π)
        const float th1 = twoPi * ((float)i / (float)N);
        // 2차 고조파는 각도가 2배
        const float th2 = 2.0f * th1 + phi;

        // 합성 (상대 진폭)
        const float s = A1_rel * sinf(th1) + A2_rel * sinf(th2);

        // 16-bit 정수로 투영 (초기 스케일은 ±30000 정도로)
        long v = lroundf(s * 30000.0f);
        v = constrain(v, -32767, 32767);
        out[i] = (int16_t)v;
    }

    if (removeDC)      removeDC_int16(out, N);
    if (normalizePeak) normalizePeak_int16(out, N);
}

/**
 * (선택) 합성 파형의 jerk 비대칭 지표를 대략적으로 계산
 * - 아주 간단히: 인접 샘플 2차 차분을 "jerk 근사"로 보고
 *   양의 피크와 음의 피크의 차이를 반환 (양수면 양/음 비대칭 존재)
 */
static float estimateJerkAsymmetry(const int16_t* w, int N) {
    if (N < 3) return 0.0f;
    float jp = 0.0f, jn = 0.0f; // positive/negative peak (절대값 최대치 추적)
    for (int i = 1; i < N - 1; ++i) {
        // 2차 차분 근사: w[i+1] - 2w[i] + w[i-1]
        const float j = (float)w[i + 1] - 2.0f * (float)w[i] + (float)w[i - 1];
        if (j > jp) jp = j;
        if (-j > jn) jn = -j;
    }
    return (jp - jn); // >0이면 양쪽 비대칭 존재(대략)
}

/**
 * (편의) 합성→즉시 재생
 * - freqHz: 재생 주파수(파형 1주기의 반복 Hz). 논문에선 75/150Hz 합성 "형태" 자체를 만들고,
 *           실제 출력은 40Hz, 10Hz 등 네 실험 주파수로 반복 재생 가능(느리게/빠르게).
 * - sec: 재생 시간
 * - gain: 네 체계의 "크기" 스케일 (DAC 출력 전에 곱해지는 값)
 */
static void playCompositeF_2F_Phase_now(
    float A1_rel, float A2_rel, float phi_deg,
    float gain, int dacPin,
    float freqHz, float sec
) {
    static int16_t buf[256];
    makeCompositeF_2F_Phase(buf, 256, A1_rel, A2_rel, phi_deg, /*removeDC=*/true, /*normalizePeak=*/true);
    playWaveForSeconds2(buf, 256, gain, dacPin, freqHz, sec);
}

//=================================================================================================================



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
            //b -> C 횡단 
            //YY
            // 
            //update hz
            updateDelayFromTargetHz();

            //for comb1 (yz) 
            const float GAIN = 3.0f; 
            for (int repeat = 0; repeat < 10; repeat++)
            {
                //Left flesh
                playArrayWithGainCentered(impulseDynamicPR, waveformSize, GAIN, DAC_PIN_A22, delayPerSampleUs_rt);
                delay(500); //(phase)

                //시간차 조정 어떻게 하냐? 
                
                //Right Flesh
                playArrayWithGainCentered(impulse_dynamic, waveformSize, GAIN, DAC_PIN_A21, delayPerSampleUs_rt);
                //하나의 pulse 이후 쉬기
                delay(500);
            }

        }

        //Practice session 
        else if (command == '1')
        {
            updateDelayFromTargetHz();
            const float GAIN = 3.0f;
            // rise:fall = 1:2
            for (int i = 0; i < 10; i++)
            {
                playArrayWithGainCentered(wave_impulseDampedTail, waveformSize, GAIN, DAC_PIN_A21, delayPerSampleUs_rt);
                delay(100);
            }
            //playHalfSineWithRatio(wave_impulseDampedTail, waveformSize, GAIN, DAC_PIN_A22,
            //    delayPerSampleUs_rt, 1.0f, 2.0f, /*repeats=*/5);
        }
        else if (command == '2')
        {
            //updateDelayFromTargetHz();
            //const float GAIN = 3.0f;
            // rise:fall = 1:3
            /*
            for (int i = 0; i < 10; i++)
            {
                playArrayWithGainCentered(impulseDampedTailPR, waveformSize, GAIN, DAC_PIN_A21, delayPerSampleUs_rt);
                delay(100);
            }
            */
            //playHalfSineWithRatio(impulseDampedTailPR, waveformSize, GAIN, DAC_PIN_A22,
            //    delayPerSampleUs_rt, 1.0f, 2.0f, /*repeats=*/5);

            float baseG_A22 = 3.0f;
            float baseG_A21 = 5.0f; // ← A21만 한 단계 더 크게
            float f = 40.0f;

            float G22 = compensatedGain(baseG_A22, f);
            float G21 = compensatedGain(baseG_A21, f);

            playArrayWithGainCentered_1cycle(wave_impulseDampedTail, 256, 3.0f, DAC_PIN_A22, 40.0f);
            delay(10);
            playWaveForSeconds2(dat, waveformSize, G21, DAC_PIN_A21, f, 0.5f);
        }
        else if (command == '3')
        {
            // 예) dat 한 주기를 2.0초 동안 늘려서 A22로 재생
            float sec = 0.5f;
            float G = 6.0f;                 // VCA면 그냥 고정게인, LRA면 f=1/sec에 맞춰 보정 필요할 수 있음
            //playWaveForSeconds2(dat, 256, G, DAC_PIN_A22, /*hz=*/1.0f / sec, /*sec=*/sec);

            //playWaveOneCycleForSeconds_Interpolated(dat, 256, /*gain=*/5.0f, DAC_PIN_A22, /*sec=*/2.0f, /*upsample=*/4);

            //hz = 40;
            // a -> d
            // repulsive force : 반대로 잡아 당김 / 진동의 방향성 -> / 움직임 right-to-left 일 때 
            //playArrayWithGainCentered(impulse_releasePR, waveformSize, G, DAC_PIN_A22, delayPerSampleUs_rt);
            playArrayWithGainCentered_1cycle(impulse_releasePR, waveformSize, G, DAC_PIN_A22, delayPerSampleUs_rt);
            //delay(10); //delay 0.01s; -> ㅂㄹ
            delay(50); //이게 중요한듯 
            //playArrayWithGainCentered(impulse_release, waveformSize, G, DAC_PIN_A21, delayPerSampleUs_rt);
            playArrayWithGainCentered_1cycle(impulse_release, waveformSize, G, DAC_PIN_A21, delayPerSampleUs_rt);
            //playWaveForSeconds2(dat, 256, G, DAC_PIN_A22, 40.0f, sec);


            // attractive force :


        }


        else if (command == '4')
        {
            updateDelayFromTargetHz();
            const float GAIN = 3.0f;
            // rise:fall = 2:1
            for (int i = 0; i < 10; i++)
            {
                playArrayWithGainCentered(impulseDampedTailPR, waveformSize, GAIN, DAC_PIN_A21, delayPerSampleUs_rt);
                delay(100);
            }
            //playHalfSineWithRatio(wave_impulseDampedTail, waveformSize, GAIN, DAC_PIN_A21,
            //    delayPerSampleUs_rt, 2.0f, 1.0f, /*repeats=*/5);
        }



        else if (command == '5')
        {
            //mpu 써서 calibration 
            //mpu_checkZAlignment(/*duration_ms=*/3000, /*sample_hz=*/500);

            //내가 궁금한거 10.0f일때랑, 40hz일때랑 intensity 같은지, 

            float baseG = 7.0f;
            float f = 40.0f;
            float G = compensatedGain(baseG, f);
            playWaveForSeconds2(dat, waveformSize, G, DAC_PIN_A22, f, 1.0f);


        }
        else if (command == '6')
        {
            //5초간 함성. 
            //mpu_streamOrientation(/*duration_ms=*/5000, /*print_hz=*/25);

            float baseG = 7.0f;
            float f = 40.0f;
            float G = compensatedGain(baseG, f);
            playWaveForSeconds2(negDatTrial, waveformSize, G, DAC_PIN_A22, f, 1.0f);

        }

        else if (command == '7')
        {
            float baseG = 7.0f;
            float f = 80.0f;
            float G = compensatedGain(baseG, f);
            playWaveForSeconds2(dat, waveformSize, G, DAC_PIN_A22, f, 1.0f);
        }

        //imu calibration 1
        else if (command == '8') {
            // 주파수 고정(예: 40Hz)에서 baseGain = 3~8까지 1초씩 측정
            static const float baseGList[] = { 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f };
            const int M = sizeof(baseGList) / sizeof(baseGList[0]);

            // 40 Hz 추이
            runFixedFreqGainSweep_MPU9250(
                dat, waveformSize, DAC_PIN_A22,
                /*freqHz=*/40.0f,
                baseGList, M,
                /*secPerGain=*/1.0f,
                /*imu_rate_hz=*/1000,
                /*warmup_s=*/0.20f,
                /*useMagnitude=*/true, /*axis=*/'z'
            );

            // 필요하면 10 Hz도 추가로
            runFixedFreqGainSweep_MPU9250(
                dat, waveformSize, DAC_PIN_A22,
                /*freqHz=*/10.0f,
                baseGList, M,
                /*secPerGain=*/1.0f,
                /*imu_rate_hz=*/1000,
                /*warmup_s=*/0.20f,
                /*useMagnitude=*/true, /*axis=*/'z'
            );
        }
        //imu calibration 2
        else if (command == '9')
        {
            static const float baseGList[] = { 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f };
            static const float freqList[] = { 10.0f, 20.0f, 30.0f, 40.0f, 60.0f, 80.0f };

            Serial.println(F("\n==== BaseGain Sweep (RAW vs COMP) ===="));

            // 필요하면 IMU 설정(ODR/DLPF)은 setup에서 이미 수행됨.
            runBaseGainSweep_MPU9250(
                /*wave=*/dat,             /*N=*/waveformSize,
                /*dacPin=*/DAC_PIN_A22,
                /*baseGList=*/baseGList,  /*M=*/(int)(sizeof(baseGList) / sizeof(baseGList[0])),
                /*freqList=*/freqList,    /*K=*/(int)(sizeof(freqList) / sizeof(freqList[0])),
                /*secPerCombo=*/1.0f,     // 각 조합 1초 측정
                /*imu_rate_hz=*/1000,     // IMU 샘플링(실제 read 주기)
                /*warmup_s=*/0.20f,       // 워밍업 구간(초): 측정에서 제외
                /*useMagnitude=*/true,    // |a| RMS 기준. 축 하나만 보려면 false로 하고 axis='z' 등
                /*axisForSingle=*/'z',
                /*alsoTestCompensated=*/true // RAW와 LUT보정 둘 다 측정
            );

            Serial.println(F("==== End of BaseGain Sweep ====\n"));
        }
        //imu calibration 3
        else if (command == 'K') 
        {
            //calibration code !!!!!!!!!!!!
            static const float freqs[] = { 10, 15, 20, 30, 40, 50, 60, 80 };
            runFreqCalibration_MPU9250(
                dat, waveformSize, DAC_PIN_A22,
                /*baseGain=*/3.0f,
                freqs, (int)(sizeof(freqs) / sizeof(freqs[0])),
                /*secPerFreq=*/1.0f,
                /*targetRMS_g=*/0.80f,
                /*imu_rate_hz=*/1000,
                /*warmup_s=*/0.20f,
                /*useMagnitude=*/true, /*axis=*/'z'
            );
        }

        //합성파형 재생
        else if (command == 'F')
        {
            playWaveForSeconds2(wPhase0, 256, 5.0f, DAC_PIN_A22, 40.0f, 1.0f);
        }
        else if (command == 'D') 
        {                 
            playWaveForSeconds2(wPhase180, 256, 5.0f, DAC_PIN_A22, 40.0f, 1.0f);
        }

        else if (command == 'S')
        {
            playAsymTimingDual(wPhase0, 256, 5.0f, 5.0f, DAC_PIN_A22, DAC_PIN_A21,
                /*delayRiseUs=*/200, /*delayFallUs=*/200, /*invertA21=*/true);
        }

        else if (command == 'A')
        {
            //A를 누르게 되면, 10hz 로 변경해서 좀 느리게 할 수 있는지 확인 
            targetHz = 10;
            updateDelayFromTargetHz();
            const float GAIN = 5.0f;

            for (int repeat = 0; repeat < 10; repeat++)
            {
                //for comb1 (yz) 
                //const float GAIN = 5.0f;
                //Left flesh
                playArrayWithGainCentered(impulse_dynamic, waveformSize, GAIN, DAC_PIN_A22, delayPerSampleUs_rt);
                delay(500); //(phase)

                //Right Flesh
                playArrayWithGainCentered(impulse_dynamic, waveformSize, GAIN, DAC_PIN_A21, delayPerSampleUs_rt);
                //하나의 pulse 이후 쉬기
                delay(500);
            }

        }


        else if (command == 'B')
        {
            targetHz = 20;
            updateDelayFromTargetHz();
            const float GAIN = 3.0f;

            for (int repeat = 0; repeat < 10; repeat++)
            {
                for (int i = 0; i < waveformSize; i++)
                {
                    int val = impulse_dynamic[i];

                    //이걸로 intensity를 결정하는 거임. 
                    //int 16 
                    int dacValue = map(val, -32767, 32767, 0, 4095);
                    //Serial.println(dacValue);

                    //앞 
                    analogWrite(DAC_PIN_A22, dacValue);
                    delayMicroseconds((int)delayPerSampleUs_rt);  //40hz
                    delay(500);

                    //앞
                    analogWrite(DAC_PIN_A21, dacValue);
                    delayMicroseconds((int)delayPerSampleUs_rt);  //40hz
                    delay(500);


                }
            }
        }

        else if (command == 'C')
        {
            //hz update
            updateDelayFromTargetHz();
            const float GAIN = 3.0f;
            // rise:fall = 2:1
            playHalfSineWithRatio(wave_quadPushLinReturn, waveformSize, GAIN, DAC_PIN_A22,
                delayPerSampleUs_rt, 2.0f, 1.0f, /*repeats=*/5);
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

