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
