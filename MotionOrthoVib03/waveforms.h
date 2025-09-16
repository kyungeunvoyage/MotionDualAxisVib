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

void CreateAllWaveforms();
void generatePositiveBiasedWaveform();
void generateNegDatTrial();

extern int16_t two_high[256];
extern int16_t high_high[256];
extern int16_t high_highTP[256];
extern int16_t high_highPR[256];


extern int16_t possible01[256];
extern int16_t peak_high[256];
extern int16_t slowReturnTwice[256];
extern int16_t slowReturnTwiceNeg[256];
extern int16_t slowReturnTwice2[256];
extern int16_t rightKickSlowReturn[256];
extern int16_t leftKickSlowReturn[256];
extern int16_t slowUpScale[256];
extern int16_t stableSteep[256];
extern int16_t cultberson_wave[256];

extern int16_t wave_slowUpHardDrop[256];
extern int16_t wave_expRiseLinFall[256];
extern int16_t wave_impulseDampedTail[256];
extern int16_t wave_asymHalfSine[256];
extern int16_t wave_amBurstAsym[256];
extern int16_t wave_quadPushLinReturn[256];


extern int16_t slowUpHardDropPR[256];
extern int16_t xpRiseLinFallPR[256];
extern int16_t impulseDampedTailPR[256];
extern int16_t asymHalfSinePR[256];
extern int16_t amBurstAsymPR[256];
extern int16_t quadPushLinReturnPR[256];

extern int16_t impulse_dynamic[256];

#endif
