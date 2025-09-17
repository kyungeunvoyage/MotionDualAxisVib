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


static uint8_t  g_mpu_addr = MPU_ADDR;
static float    g_accel_lsb_per_g = 4096.0f; // 기본 ±8g
static float    g_accel_g_per_lsb = 1.0f / 4096.0f;

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
    if (!i2cWriteByte(g_mpu_addr, REG_PWR_MGMT_1, 0x01)) return false; // CLK=PLL
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
//struct FG { float f; float g; };
FG kGainLUT[] = {
    // ← 캘리브레이션 끝나면 Serial 출력된 값들로 교체하세요.
    {10.0f, 1.0f},
    {20.0f, 1.0f},
    {30.0f, 1.0f},
    {40.0f, 1.0f},
    {60.0f, 1.0f},
    {80.0f, 1.0f},
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
/***** ================== (끝) MPUsetting.ino ================== *****/
