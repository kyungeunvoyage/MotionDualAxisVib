/***** ================== MPUsetting.ino ================== *****/
#include <Arduino.h>
#include <Wire.h>
#include "MPUsetting.h"

/*  메인 탭에 정의된 함수/변수 사용 선언  */
extern int toDac_12bit_centered(int16_t v);  // 메인 탭의 DAC 매핑 함수
// 재생 함수가 메인 탭에만 있다면 필요 시 이렇게 extern 선언해서 호출 가능:
// extern void playWaveForSeconds2(const int16_t* w, int N, float gain, int dacPin, float hz, float sec);

/* ================== MPU-9250 (MPU-6500) 레지스터 ================== */
#define MPU_ADDR             0x68
#define REG_PWR_MGMT_1       0x6B
#define REG_SMPLRT_DIV       0x19
#define REG_CONFIG           0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_ACCEL_CONFIG     0x1C
#define REG_ACCEL_CONFIG2    0x1D
#define REG_INT_PIN_CFG      0x37
#define REG_INT_ENABLE       0x38
#define REG_ACCEL_XOUT_H     0x3B

static uint8_t g_mpu_addr = MPU_ADDR;
static float   g_accel_lsb_per_g = 4096.0f;
static float   g_accel_g_per_lsb = 1.0f / 4096.0f;

/* ================== I2C helpers ================== */
static inline bool i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t data) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(data);
    return (Wire.endTransmission() == 0);
}
static inline bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((int)addr, (int)len);
    for (size_t i = 0; i < len && Wire.available(); ++i) buf[i] = Wire.read();
    return (Wire.available() == 0);
}

/* ================== MPU-9250 가속도 시작 ==================
 * accel_fs: ACCEL_FS_* (±2/4/8/16 g)
 * dlpf   : ACCEL_CONFIG2 DLPF(0~5). 0:~218Hz,1:~99Hz,2:~45Hz,3:~21Hz,4:~10Hz,5:~5Hz
 * odr_hz: 샘플레이트 목표(대략). 1000/500/250/200/100 등
 */
bool mpu9250_beginAccel(uint8_t accel_fs, uint8_t dlpf, uint16_t odr_hz) {
    delay(10);
    if (!i2cWriteByte(g_mpu_addr, REG_PWR_MGMT_1, 0x01)) return false;
    delay(10);

    // Gyro DLPF(ODR 영향): 0x02 ≈ 92Hz 대역
    if (!i2cWriteByte(g_mpu_addr, REG_CONFIG, 0x02)) return false;

    // Sample Rate Divider (기준 1kHz)
    uint8_t div = 0;
    if (odr_hz >= 1000) div = 0;
    else if (odr_hz >= 500) div = 1;
    else if (odr_hz >= 250) div = 3;
    else if (odr_hz >= 200) div = 4;
    else if (odr_hz >= 100) div = 9;
    else div = 19; // ~50Hz
    if (!i2cWriteByte(g_mpu_addr, REG_SMPLRT_DIV, div)) return false;

    // Accel FS
    uint8_t accel_cfg = 0;
    switch (accel_fs) {
    case ACCEL_FS_2G:  accel_cfg = 0 << 3; g_accel_lsb_per_g = 16384.0f; break;
    case ACCEL_FS_4G:  accel_cfg = 1 << 3; g_accel_lsb_per_g = 8192.0f;  break;
    case ACCEL_FS_8G:  accel_cfg = 2 << 3; g_accel_lsb_per_g = 4096.0f;  break;
    case ACCEL_FS_16G: accel_cfg = 3 << 3; g_accel_lsb_per_g = 2048.0f;  break;
    }
    g_accel_g_per_lsb = 1.0f / g_accel_lsb_per_g;
    if (!i2cWriteByte(g_mpu_addr, REG_ACCEL_CONFIG, accel_cfg)) return false;

    // Accel DLPF
    uint8_t accel_cfg2 = (dlpf & 0x07);
    if (!i2cWriteByte(g_mpu_addr, REG_ACCEL_CONFIG2, accel_cfg2)) return false;

    // INT off
    i2cWriteByte(g_mpu_addr, REG_INT_ENABLE, 0x00);

    Serial.print(F("[MPU9250] accel FS=±"));
    Serial.print((accel_fs == 0) ? 2 : ((accel_fs == 1) ? 4 : ((accel_fs == 2) ? 8 : 16)));
    Serial.print(F(" g, DLPF=")); Serial.print(dlpf);
    Serial.print(F(", ODR≈"));
    Serial.print((int)(1000 / (1 + div)));
    Serial.println(F(" Hz"));
    return true;
}

//z축 정렬 check
static inline float clampf(float x, float a, float b) {
    if (x < a) x = a;
    if (x > b) x = b;
    return x;
}
// deg 변환(Arduino에 degrees()가 있지만, 호환 위해 직접 사용)
static inline float rad2deg(float r) { return r * 57.2957795f; }

/* ================== 가속도 읽기 (g 단위) ================== */
bool mpu9250_readAccelG(float& ax_g, float& ay_g, float& az_g) {
    uint8_t buf[6];
    if (!i2cReadBytes(g_mpu_addr, REG_ACCEL_XOUT_H, buf, 6)) return false;
    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
    ax_g = (float)ax * g_accel_g_per_lsb;
    ay_g = (float)ay * g_accel_g_per_lsb;
    az_g = (float)az * g_accel_g_per_lsb;
    return true;
}
float mpu9250_readAccelMagG() {
    float ax, ay, az;
    if (!mpu9250_readAccelG(ax, ay, az)) return 0.0f;
    return sqrtf(ax * ax + ay * ay + az * az);
}
float mpu9250_readAccelAxisG(char axis) {
    float ax, ay, az;
    if (!mpu9250_readAccelG(ax, ay, az)) return 0.0f;
    switch (axis) {
    case 'x': case 'X': return fabsf(ax);
    case 'y': case 'Y': return fabsf(ay);
    default:            return fabsf(az);
    }
}

/* ================== 주파수-보정 LUT & 보간 ================== */
struct FG { float f; float g; };
FG kGainLUT[] = {
  {10.0f, 2.315490f},
  {15.0f, 2.300659f},
  {20.0f, 2.270110f},
  {30.0f, 2.102659f},
  {40.0f, 1.949178f},
  {50.0f, 1.878106f},
  {60.0f, 1.922439f},
  {80.0f, 2.037328f},
};
int kGainLUT_N = sizeof(kGainLUT) / sizeof(kGainLUT[0]);


float interpGainLUT(float f) {
    if (kGainLUT_N <= 0) return 1.0f;
    if (f <= kGainLUT[0].f) return kGainLUT[0].g;
    if (f >= kGainLUT[kGainLUT_N - 1].f) return kGainLUT[kGainLUT_N - 1].g;
    for (int i = 0; i < kGainLUT_N - 1; ++i) {
        if (f >= kGainLUT[i].f && f <= kGainLUT[i + 1].f) {
            float t = (f - kGainLUT[i].f) / (kGainLUT[i + 1].f - kGainLUT[i].f);
            return kGainLUT[i].g * (1.0f - t) + kGainLUT[i + 1].g * t;
        }
    }
    return 1.0f;
}
float compensatedGain(float baseGain, float freqHz) {
    return baseGain * interpGainLUT(freqHz);
}

/* ================== 오프라인 캘리브레이션 ==================
 * 동일 파형을 각 주파수에서 baseGain으로 secPerFreq만큼 재생하며
 * IMU로 |a|(또는 단일축) RMS(g)를 측정 → 목표 targetRMS_g로 보정 gain 산출
 * 결과를 CSV + LUT 형식으로 시리얼에 출력
 */
void runFreqCalibration_MPU9250(
    const int16_t* wave, int N, int dacPin, float baseGain,
    const float* freqList, int K, float secPerFreq, float targetRMS_g,
    uint32_t imu_rate_hz, float warmup_s, bool useMagnitude, char axisForSingle
) {
    if (K <= 0 || !wave || N <= 0) return;

    long sum = 0; for (int i = 0; i < N; ++i) sum += wave[i];
    const float mean = (float)sum / (float)N;

    Serial.println(F("# f_Hz,meas_RMS_g,compGain,finalGain(base*comp),clips_%"));
    Serial.println(F("\n// ---- Suggested LUT (freq->gain) ----"));
    Serial.println(F("struct FG { float f; float g; };"));
    Serial.println(F("static FG kGainLUT[] = {"));

    for (int k = 0; k < K; ++k) {
        const float f = freqList[k];
        if (f <= 0.0f) { Serial.println(F("// skip non-positive f")); continue; }

        const double T_us = 1000000.0 / (double)f;
        const double dt_dac = T_us / (double)N;
        const uint32_t dur_total_us = (uint32_t)llround(secPerFreq * 1000000.0);
        const uint32_t dur_warmup_us = (uint32_t)llround(warmup_s * 1000000.0);
        const uint32_t imu_period_us = (imu_rate_hz > 0) ? (1000000UL / imu_rate_hz) : 1000UL;

        const uint32_t t0 = micros();
        uint32_t next_dac = t0;
        uint32_t next_imu = t0;
        const uint32_t t_warm_end = t0 + dur_warmup_us;
        const uint32_t t_end = t0 + dur_total_us;

        int idx = 0;
        double accSq = 0.0;
        uint32_t accCount = 0;
        uint32_t clipCount = 0;

        while ((int32_t)(micros() - t0) < (int32_t)dur_total_us) {
            uint32_t now = micros();

            // DAC 출력
            if ((int32_t)(now - next_dac) >= 0) {
                float v = (wave[idx] - mean) * baseGain;
                long  v16 = lroundf(v);
                if (v16 > 32767) { v16 = 32767; clipCount++; }
                if (v16 < -32767) { v16 = -32767; clipCount++; }
                analogWrite(dacPin, toDac_12bit_centered((int16_t)v16));
                idx = (idx + 1) % N;
                next_dac += (uint32_t)llround(dt_dac);
            }

            // IMU 측정
            if ((int32_t)(now - next_imu) >= 0) {
                float a_g = useMagnitude ? mpu9250_readAccelMagG() : mpu9250_readAccelAxisG(axisForSingle);
                if ((int32_t)(now - t_warm_end) >= 0) { accSq += (double)a_g * (double)a_g; accCount++; }
                next_imu += imu_period_us;
            }
        }

        const float measRMS_g = (accCount > 0) ? (float)sqrt(accSq / (double)accCount) : 0.0f;
        const float compGain = (measRMS_g > 1e-6f) ? (targetRMS_g / measRMS_g) : 1.0f;
        const float finalGain = baseGain * compGain;

        // 대략적 클리핑률(%) — 참조용
        const float samples_meas = (float)((dur_total_us - dur_warmup_us) / (uint32_t)llround(dt_dac) + 1);
        const float clipsPct = (samples_meas > 0) ? (100.0f * (float)clipCount / samples_meas) : 0.0f;

        // CSV
        Serial.print(f, 3); Serial.print(',');
        Serial.print(measRMS_g, 6); Serial.print(',');
        Serial.print(compGain, 6);  Serial.print(',');
        Serial.print(finalGain, 6); Serial.print(',');
        Serial.println(clipsPct, 3);

        // LUT 항목
        Serial.print(F("  { ")); Serial.print(f, 3);
        Serial.print(F(", "));  Serial.print(finalGain, 6);
        Serial.println(F(" },"));
    }

    Serial.println(F("};"));
    Serial.println(F("static int kGainLUT_N = sizeof(kGainLUT)/sizeof(kGainLUT[0]);"));
    Serial.println(F("// ↑ 이 블록을 이 파일의 kGainLUT에 복붙해 런타임 보정에 사용하세요."));
}


// pitch/roll 계산(항공 좌표계 관용식)
// pitch = 앞으로 숙임(+) / roll = 오른쪽으로 기움(+)
static void computePitchRoll(float ax, float ay, float az, float& pitch_deg, float& roll_deg) {
    // 가속도 기준 자세 추정: 흔히 쓰는 근사식
    float pitch = atanf(-ax / sqrtf(ay * ay + az * az));
    float roll = atanf(ay / (az == 0 ? 1e-6f : az));
    pitch_deg = rad2deg(pitch);
    roll_deg = rad2deg(roll);
}

// duration_ms 동안 sample_hz로 읽어서 평균값을 내고, Z축과 중력의 각도를 출력
void mpu_checkZAlignment(uint16_t duration_ms, uint16_t sample_hz) {
    if (sample_hz == 0) sample_hz = 500;
    uint32_t period_us = 1000000UL / sample_hz;
    uint32_t t0 = micros();
    uint32_t tend = t0 + (uint32_t)duration_ms * 1000UL;

    double sumx = 0.0, sumy = 0.0, sumz = 0.0;
    uint32_t count = 0;
    uint32_t next_t = t0;

    while ((int32_t)(micros() - t0) < (int32_t)duration_ms * 1000) {
        uint32_t now = micros();
        if ((int32_t)(now - next_t) >= 0) {
            float ax, ay, az;
            if (mpu9250_readAccelG(ax, ay, az)) {
                sumx += ax; sumy += ay; sumz += az;
                count++;
            }
            next_t += period_us;
        }
    }

    if (count == 0) {
        Serial.println(F("[IMU] No samples read."));
        return;
    }

    float ax = (float)(sumx / (double)count);
    float ay = (float)(sumy / (double)count);
    float az = (float)(sumz / (double)count);
    float amag = sqrtf(ax * ax + ay * ay + az * az);

    // Z축과 중력(평균 가속도) 사이 각도
    // cos(theta) = az / |a|
    float cosZ = (amag > 1e-6f) ? (az / amag) : 0.0f;
    cosZ = clampf(cosZ, -1.0f, 1.0f);
    float thetaZ_deg = rad2deg(acosf(cosZ));

    float pitch_deg, roll_deg;
    computePitchRoll(ax, ay, az, pitch_deg, roll_deg);

    Serial.println(F("==== IMU Z-Alignment Check ===="));
    Serial.print(F("Duration(ms): ")); Serial.print(duration_ms);
    Serial.print(F(", SampleHz: "));   Serial.println(sample_hz);

    Serial.print(F("Avg ax,ay,az (g): "));
    Serial.print(ax, 4); Serial.print(F(", "));
    Serial.print(ay, 4); Serial.print(F(", "));
    Serial.println(az, 4);

    Serial.print(F("|a| (g): ")); Serial.println(amag, 4);

    Serial.print(F("Angle between +Z and gravity (deg): "));
    Serial.println(thetaZ_deg, 2);

    Serial.print(F("Pitch (deg): ")); Serial.print(pitch_deg, 2);
    Serial.print(F(" , Roll (deg): ")); Serial.println(roll_deg, 2);

    // 간단 판정: 10° 이하면 거의 평행, 10~25° 주의, 그 이상은 재정렬 권장
    if (thetaZ_deg <= 10.0f) {
        Serial.println(F("[OK] Z-axis is nearly parallel to gravity (<=10°)."));
    }
    else if (thetaZ_deg <= 25.0f) {
        Serial.println(F("[WARN] Z-axis somewhat tilted (10~25°). Consider minor adjustment."));
    }
    else {
        Serial.println(F("[NG] Z-axis is far from gravity direction (>25°). Reposition IMU."));
    }

    Serial.println(F("================================"));
}

// 간단 실시간 스트리밍: 주기적으로 ax,ay,az,|a|,thetaZ를 텍스트로 출력
void mpu_streamOrientation(uint16_t duration_ms, uint16_t print_hz) {
    if (print_hz == 0) print_hz = 25;
    uint32_t period_us = 1000000UL / print_hz;
    uint32_t t0 = micros();
    uint32_t next_t = t0;

    Serial.println(F("# t_ms, ax_g, ay_g, az_g, |a|_g, thetaZ_deg"));

    while ((int32_t)(micros() - t0) < (int32_t)duration_ms * 1000) {
        uint32_t now = micros();
        if ((int32_t)(now - next_t) >= 0) {
            float ax, ay, az;
            if (mpu9250_readAccelG(ax, ay, az)) {
                float amag = sqrtf(ax * ax + ay * ay + az * az);
                float cosZ = (amag > 1e-6f) ? (az / amag) : 0.0f;
                cosZ = clampf(cosZ, -1.0f, 1.0f);
                float thetaZ_deg = rad2deg(acosf(cosZ));

                uint32_t t_ms = (micros() - t0) / 1000UL;
                Serial.print(t_ms); Serial.print(',');
                Serial.print(ax, 4); Serial.print(',');
                Serial.print(ay, 4); Serial.print(',');
                Serial.print(az, 4); Serial.print(',');
                Serial.print(amag, 4); Serial.print(',');
                Serial.println(thetaZ_deg, 2);
            }
            next_t += period_us;
        }
    }
    Serial.println(F("# stream end"));
}

// MPUsetting.ino

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
) {
    if (!wave || N <= 0 || M <= 0 || K <= 0) return;

    // DC 제거용 평균 (전 구간 고정)
    long sum = 0; for (int i = 0; i < N; ++i) sum += wave[i];
    const float mean = (float)sum / (float)N;

    // 헤더
    Serial.println(F("#MODE,baseG,f_Hz,gainApplied,meas_RMS_g,clips_%"));
    Serial.println(F("# MODE: RAW(보정없음), COMP(LUT보정사용)"));

    for (int mi = 0; mi < M; ++mi) {
        const float baseG = baseGList[mi];

        for (int ki = 0; ki < K; ++ki) {
            const float f = freqList[ki];
            if (f <= 0.0f) continue;

            // 공통 타이밍 파라미터
            const double T_us = 1000000.0 / (double)f;   // 1사이클(파형) 시간
            const double dt_dac = T_us / (double)N;        // 샘플 간 간격
            const uint32_t dur_total_us = (uint32_t)llround(secPerCombo * 1000000.0);
            const uint32_t dur_warmup_us = (uint32_t)llround(warmup_s * 1000000.0);
            const uint32_t imu_period_us = (imu_rate_hz > 0) ? (1000000UL / imu_rate_hz) : 1000UL;

            // ① RAW (보정 미적용)
            {
                const float gainApplied = baseG;

                const uint32_t t0 = micros();
                uint32_t next_dac = t0;
                uint32_t next_imu = t0;
                const uint32_t t_warm_end = t0 + dur_warmup_us;

                int idx = 0;
                double accSq = 0.0;
                uint32_t accCount = 0;
                uint32_t clipCount = 0;

                while ((int32_t)(micros() - t0) < (int32_t)dur_total_us) {
                    uint32_t now = micros();

                    // DAC 출력
                    if ((int32_t)(now - next_dac) >= 0) {
                        float v = (wave[idx] - mean) * gainApplied;
                        long  v16 = lroundf(v);
                        if (v16 > 32767) { v16 = 32767; clipCount++; }
                        if (v16 < -32767) { v16 = -32767; clipCount++; }
                        analogWrite(dacPin, toDac_12bit_centered((int16_t)v16));
                        idx = (idx + 1) % N;
                        next_dac += (uint32_t)llround(dt_dac);
                    }

                    // IMU 샘플
                    if ((int32_t)(now - next_imu) >= 0) {
                        float a_g = useMagnitude ? mpu9250_readAccelMagG()
                            : mpu9250_readAccelAxisG(axisForSingle);
                        if ((int32_t)(now - t_warm_end) >= 0) { accSq += (double)a_g * (double)a_g; accCount++; }
                        next_imu += imu_period_us;
                    }
                }

                const float measRMS_g = (accCount > 0) ? (float)sqrt(accSq / (double)accCount) : 0.0f;
                const float samples_meas = (float)((dur_total_us - dur_warmup_us) / (uint32_t)llround(dt_dac) + 1);
                const float clipsPct = (samples_meas > 0) ? (100.0f * (float)clipCount / samples_meas) : 0.0f;

                Serial.print(F("RAW,"));
                Serial.print(baseG, 6);          Serial.print(',');
                Serial.print(f, 3);              Serial.print(',');
                Serial.print(gainApplied, 6);    Serial.print(',');
                Serial.print(measRMS_g, 6);      Serial.print(',');
                Serial.println(clipsPct, 3);
            }

            // ② COMP (주파수 보정 적용)
            if (alsoTestCompensated) {
                const float gainApplied = compensatedGain(baseG, f);

                const uint32_t t0 = micros();
                uint32_t next_dac = t0;
                uint32_t next_imu = t0;
                const uint32_t t_warm_end = t0 + dur_warmup_us;

                int idx = 0;
                double accSq = 0.0;
                uint32_t accCount = 0;
                uint32_t clipCount = 0;

                while ((int32_t)(micros() - t0) < (int32_t)dur_total_us) {
                    uint32_t now = micros();

                    // DAC 출력
                    if ((int32_t)(now - next_dac) >= 0) {
                        float v = (wave[idx] - mean) * gainApplied;
                        long  v16 = lroundf(v);
                        if (v16 > 32767) { v16 = 32767; clipCount++; }
                        if (v16 < -32767) { v16 = -32767; clipCount++; }
                        analogWrite(dacPin, toDac_12bit_centered((int16_t)v16));
                        idx = (idx + 1) % N;
                        next_dac += (uint32_t)llround(dt_dac);
                    }

                    // IMU 샘플
                    if ((int32_t)(now - next_imu) >= 0) {
                        float a_g = useMagnitude ? mpu9250_readAccelMagG()
                            : mpu9250_readAccelAxisG(axisForSingle);
                        if ((int32_t)(now - t_warm_end) >= 0) { accSq += (double)a_g * (double)a_g; accCount++; }
                        next_imu += imu_period_us;
                    }
                }

                const float measRMS_g = (accCount > 0) ? (float)sqrt(accSq / (double)accCount) : 0.0f;
                const float samples_meas = (float)((dur_total_us - dur_warmup_us) / (uint32_t)llround(dt_dac) + 1);
                const float clipsPct = (samples_meas > 0) ? (100.0f * (float)clipCount / samples_meas) : 0.0f;

                Serial.print(F("COMP,"));
                Serial.print(baseG, 6);          Serial.print(',');
                Serial.print(f, 3);              Serial.print(',');
                Serial.print(gainApplied, 6);    Serial.print(',');
                Serial.print(measRMS_g, 6);      Serial.print(',');
                Serial.println(clipsPct, 3);
            }
        }
    }

    Serial.println(F("#DONE BaseGainSweep"));
}

void runFixedFreqGainSweep_MPU9250(
    const int16_t* wave, int N, int dacPin,
    float freqHz,
    const float* baseGains, int M,
    float secPerGain,
    uint32_t imu_rate_hz,
    float warmup_s,
    bool useMagnitude,
    char axisForSingle
)
{
    if (!wave || N <= 0 || M <= 0 || freqHz <= 0.0f) return;

    // DC 제거(평균) 미리 계산
    long sum = 0; for (int i = 0; i < N; ++i) sum += wave[i];
    const float mean = (float)sum / (float)N;

    // 타이밍 파라미터
    const double T_us = 1000000.0 / (double)freqHz;
    const double dt_dac = T_us / (double)N;
    const uint32_t imu_dt_us = (imu_rate_hz > 0) ? (1000000UL / imu_rate_hz) : 1000UL;

    Serial.println();
    Serial.println(F("### Fixed-Frequency Gain Sweep"));
    Serial.print(F("# freqHz=")); Serial.println(freqHz, 3);
    Serial.println(F("# baseGain, measRMS_g, clips_%"));

    for (int gidx = 0; gidx < M; ++gidx) {
        const float G = baseGains[gidx];
        const uint32_t dur_total_us = (uint32_t)llround(secPerGain * 1000000.0);
        const uint32_t dur_warmup_us = (uint32_t)llround(warmup_s * 1000000.0);

        const uint32_t t0 = micros();
        uint32_t next_dac = t0;
        uint32_t next_imu = t0;
        const uint32_t t_warm_end = t0 + dur_warmup_us;

        int idx = 0;
        double accSq = 0.0;
        uint32_t accCount = 0;
        uint32_t clipCount = 0;

        while ((int32_t)(micros() - t0) < (int32_t)dur_total_us) {
            uint32_t now = micros();

            // DAC 출력
            if ((int32_t)(now - next_dac) >= 0) {
                float v = (wave[idx] - mean) * G;
                long  v16 = lroundf(v);
                if (v16 > 32767) { v16 = 32767; clipCount++; }
                if (v16 < -32767) { v16 = -32767; clipCount++; }
                analogWrite(dacPin, toDac_12bit_centered((int16_t)v16));
                idx = (idx + 1) % N;
                next_dac += (uint32_t)llround(dt_dac);
            }

            // IMU 측정
            if ((int32_t)(now - next_imu) >= 0) {
                float a_g = useMagnitude ? mpu9250_readAccelMagG() : mpu9250_readAccelAxisG(axisForSingle);
                if ((int32_t)(now - t_warm_end) >= 0) { accSq += (double)a_g * (double)a_g; accCount++; }
                next_imu += imu_dt_us;
            }
        }

        const float measRMS_g = (accCount > 0) ? (float)sqrt(accSq / (double)accCount) : 0.0f;

        // 대략적 클리핑률(%) — DAC 출력 스텝 기준의 근사
        const float samples_meas = (float)((dur_total_us - dur_warmup_us) / (uint32_t)llround(dt_dac) + 1);
        const float clipsPct = (samples_meas > 0) ? (100.0f * (float)clipCount / samples_meas) : 0.0f;

        Serial.print(G, 3); Serial.print(',');
        Serial.print(measRMS_g, 6); Serial.print(',');
        Serial.println(clipsPct, 3);
    }

    //erial.println(F("# (Tip) 위 CSV를 복사해 엑셀/파이썬에서 baseGain-세기 곡선 그려보면 추세 확인 쉬움"));
}
