 /*
 Name:		ExploratoryStudy.ino
 Created:	2025-08-22 오후 3:11:20
 Author:	HCITECH_01
*/

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

//==================VCA setup====================
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
//===============================================

// the setup function runs once when you press reset or power the board
void setup() {
	Wire.begin();
	Serial.begin(115200);
	while (!Serial);
	delay(100);

	Serial.println("Initiated");

	generateNegDatTrial();

	CreateAllWaveforms();  // << 여기서 한 번에 생성!
	generatePositiveBiasedWaveform();

	//time phase 
	for (int i = 0; i < 256; i++) high_highTP[i] = high_high[255 - i];

	//Polarity reverse
	for (int i = 0; i < 256; i++) high_highPR[i] = -high_high[i];

}


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

//================arrange TargetHz Setting=========
void updateDelayFromTargetHz() {
	float cycleDurationMs = 1000.0 / targetHz;
	delayPerSampleUs_rt = (cycleDurationMs * 1000.0) / waveformSize;
	//Serial.print("Updated delayPerSampleUs: ");
	//Serial.println(delayPerSampleUs);
}
//==================================================

//======================findPeak====================
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
//====================================================

////==============DUO play==========================
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

//===========================Polarity Reverse가 포함된 함수======================
//반대도 플레이하는 경우의수 
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
//====================================================================


// the loop function runs over and over again until power down or reset
void loop() {
  
	if (Serial.available() > 0)
	{
		char command = Serial.read();
		if (command == '0')
		{
			updateDelayFromTargetHz();
			//그냥 단순 play 
			const float GAIN = 3.0f;
			for (int repeat = 0; repeat < 50; repeat++)
			{
				//이건 + 방향이니까 반대도 해줘야지 
				playArrayWithGainCentered(wave_slowUpHardDrop, waveformSize, GAIN, DAC_PIN_A22, delayPerSampleUs_rt);
				delay(100);
				Serial.println(repeat + "repeat");
			}

		}
	}


}
