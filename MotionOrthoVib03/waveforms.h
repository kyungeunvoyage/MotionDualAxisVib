// waveforms.h

#ifndef _WAVEFORMS_h
#define _WAVEFORMS_h

#include <Arduino.h>

// 배열 선언
extern int16_t dat[256];
extern int16_t datCentered[170];
extern int16_t dat2[256];
extern int16_t asyTriangular[256];

// 생성된 waveform 저장 배열
extern int16_t newWave_0[256];
extern int16_t newWave_1[256];
extern int16_t newWave_2[256];
extern int16_t newWave_3[256];
extern int16_t newWave_4[256];
extern int16_t newWave_5[256];
extern int16_t newWave_6[256];
extern int16_t biasWaveform[256];
extern int16_t negDatTrial[256];

// 함수 선언
void CreateAllWaveforms();
void generatePositiveBiasedWaveform();
void generateNegDatTrial();

#endif