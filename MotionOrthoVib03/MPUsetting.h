#pragma once
#include <Arduino.h>

// ==== Accel FS enum (헤더에 있어야 메인에서 보임) ====
enum AccelFS {
    ACCEL_FS_2G = 0,
    ACCEL_FS_4G = 1,
    ACCEL_FS_8G = 2,
    ACCEL_FS_16G = 3
};

// ==== 함수 선언(기본 인자는 '헤더'에만 둡니다) ====
bool  mpu9250_beginAccel(uint8_t accel_fs = ACCEL_FS_8G,
    uint8_t dlpf = 2,
    uint16_t odr_hz = 1000);
bool  mpu9250_readAccelG(float& ax_g, float& ay_g, float& az_g);
float mpu9250_readAccelMagG();
float mpu9250_readAccelAxisG(char axis = 'z'); // 'z' 처럼 작은따옴표!

// ==== 보정 LUT (정의는 .ino에, 여기엔 extern만) ====
struct FG { float f; float g; };
extern FG  kGainLUT[];
extern int kGainLUT_N;

float interpGainLUT(float f);
float compensatedGain(float baseGain, float freqHz);

// ==== 오프라인 캘리브 ====
void runFreqCalibration_MPU9250(
    const int16_t* wave, int N, int dacPin, float baseGain,
    const float* freqList, int K, float secPerFreq, float targetRMS_g,
    uint32_t imu_rate_hz = 1000, float warmup_s = 0.20f,
    bool useMagnitude = true, char axisForSingle = 'z'
);

// Z축-중력 평행도 체크(평균 기반)
void mpu_checkZAlignment(uint16_t duration_ms = 3000, uint16_t sample_hz = 500);

// 실시간 스트리밍(간이 모니터)
void mpu_streamOrientation(uint16_t duration_ms = 3000, uint16_t print_hz = 25);

//실제 진동 세기 (가속도 RMS) 측정 
void runBaseGainSweep_MPU9250(
    const int16_t* wave, int N, int dacPin,
    const float* baseGList, int M,
    const float* freqList, int K,
    float secPerCombo,
    uint32_t imu_rate_hz,
    float warmup_s,
    bool useMagnitude,
    char axisForSingle,
    bool alsoTestCompensated
);

void runFixedFreqGainSweep_MPU9250(
    const int16_t* wave, int N, int dacPin,
    float freqHz,
    const float* baseGains, int M,
    float secPerGain,
    uint32_t imu_rate_hz,
    float warmup_s,
    bool useMagnitude,
    char axisForSingle
);