// waveforms.h

#ifndef _WAVEFORMS_h
#define _WAVEFORMS_h
#include <Arduino.h>

// �迭 ����
extern int16_t dat[256];
extern int16_t datCentered[170];
extern int16_t dat2[256];
extern int16_t asyTriangular[256];

// ������ waveform ���� �迭
extern int16_t newWave_0[256];
extern int16_t newWave_1[256];
extern int16_t newWave_2[256];
extern int16_t newWave_3[256];
extern int16_t newWave_4[256];
extern int16_t newWave_5[256];
extern int16_t newWave_6[256];
extern int16_t biasWaveform[256];
extern int16_t negDatTrial[256];

extern int16_t newWave_custom[256];

// �Լ� ����
void CreateAllWaveforms();
void generatePositiveBiasedWaveform();
void generateNegDatTrial();

extern int16_t two_high[256];
extern int16_t high_high[256];
extern int16_t possible01[256];
#endif
