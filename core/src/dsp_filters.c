#include "dsp_filters.h"

#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979323846f

void biquad_init_lpf(biquad_filter_t *f, float sample_hz, float cutoff_hz)
{
    if (f == NULL || sample_hz <= 0.0f || cutoff_hz <= 0.0f) {
        return;
    }
    memset(f, 0, sizeof(*f));
    float omega = 2.0f * PI_F * cutoff_hz / sample_hz;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * 0.70710678f);
    float a0 = 1.0f + alpha;

    f->b0 = ((1.0f - cs) * 0.5f) / a0;
    f->b1 = (1.0f - cs) / a0;
    f->b2 = ((1.0f - cs) * 0.5f) / a0;
    f->a1 = (-2.0f * cs) / a0;
    f->a2 = (1.0f - alpha) / a0;
}

void biquad_update_notch_coeffs(biquad_filter_t *f, float sample_hz, float center_hz, float q_factor)
{
    if (f == NULL || sample_hz <= 0.0f) {
        return;
    }
    float max_fc = sample_hz * 0.45f;
    if (center_hz < 30.0f) center_hz = 30.0f;
    if (center_hz > max_fc) center_hz = max_fc;
    if (q_factor < 0.5f) q_factor = 0.5f;

    float omega = 2.0f * PI_F * center_hz / sample_hz;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * q_factor);
    float a0 = 1.0f + alpha;

    f->b0 = 1.0f / a0;
    f->b1 = (-2.0f * cs) / a0;
    f->b2 = 1.0f / a0;
    f->a1 = (-2.0f * cs) / a0;
    f->a2 = (1.0f - alpha) / a0;
}

void biquad_init_notch(biquad_filter_t *f, float sample_hz, float center_hz, float q_factor)
{
    if (f == NULL) {
        return;
    }
    memset(f, 0, sizeof(*f));
    biquad_update_notch_coeffs(f, sample_hz, center_hz, q_factor);
}

float biquad_apply(biquad_filter_t *f, float x)
{
    if (f == NULL) {
        return x;
    }
    float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;
    f->x2 = f->x1;
    f->x1 = x;
    f->y2 = f->y1;
    f->y1 = y;
    return y;
}

/* In-place 64-point Radix-2 Cooley-Tukey FFT returning dominant peak bin frequency */
static float compute_fft64_peak_hz(const float *samples, float effective_fs_hz)
{
    float re[DSP_FFT_N];
    float im[DSP_FFT_N];

    for (size_t i = 0U; i < DSP_FFT_N; i++) {
        /* Hann window to minimize spectral leakage */
        float win = 0.5f * (1.0f - cosf((2.0f * PI_F * (float)i) / (float)(DSP_FFT_N - 1U)));
        re[i] = samples[i] * win;
        im[i] = 0.0f;
    }

    /* Bit-reversal permutation */
    size_t j = 0U;
    for (size_t i = 0U; i < DSP_FFT_N; i++) {
        if (i < j) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
        size_t bit = DSP_FFT_N >> 1U;
        while (bit > 0U && (j & bit) != 0U) {
            j ^= bit;
            bit >>= 1U;
        }
        j |= bit;
    }

    /* Butterfly stages */
    for (size_t len = 2U; len <= DSP_FFT_N; len <<= 1U) {
        float ang = -2.0f * PI_F / (float)len;
        float wlen_r = cosf(ang);
        float wlen_i = sinf(ang);
        for (size_t i = 0U; i < DSP_FFT_N; i += len) {
            float wr = 1.0f;
            float wi = 0.0f;
            for (size_t k = 0U; k < len / 2U; k++) {
                size_t u = i + k;
                size_t v = i + k + len / 2U;
                float tr = re[v] * wr - im[v] * wi;
                float ti = re[v] * wi + im[v] * wr;
                re[v] = re[u] - tr;
                im[v] = im[u] - ti;
                re[u] += tr;
                im[u] += ti;
                float next_wr = wr * wlen_r - wi * wlen_i;
                wi = wr * wlen_i + wi * wlen_r;
                wr = next_wr;
            }
        }
    }

    /* Search frequency bins above 30Hz (avoid DC/pilot maneuvers) up to Nyquist */
    size_t best_bin = 4U;
    float best_mag2 = 0.0f;
    for (size_t b = 4U; b < DSP_FFT_N / 2U; b++) {
        float mag2 = re[b] * re[b] + im[b] * im[b];
        if (mag2 > best_mag2) {
            best_mag2 = mag2;
            best_bin = b;
        }
    }

    return ((float)best_bin * effective_fs_hz) / (float)DSP_FFT_N;
}

void dsp_filter_bank_init(dsp_filter_bank_t *bank,
                          float sample_rate_hz,
                          float gyro_lpf_hz,
                          float accel_lpf_hz)
{
    if (bank == NULL) {
        return;
    }
    memset(bank, 0, sizeof(*bank));
    bank->sample_rate_hz = (sample_rate_hz > 50.0f) ? sample_rate_hz : 500.0f;
    bank->notch1_hz = 120.0f;
    bank->notch2_hz = 220.0f;
    bank->fft_peak_hz = 140.0f;

    for (size_t i = 0U; i < 3U; i++) {
        biquad_init_lpf(&bank->gyro_lpf[i], bank->sample_rate_hz, gyro_lpf_hz);
        biquad_init_lpf(&bank->accel_lpf[i], bank->sample_rate_hz, accel_lpf_hz);
        biquad_init_notch(&bank->gyro_notch1[i], bank->sample_rate_hz, bank->notch1_hz, 4.0f);
        biquad_init_notch(&bank->gyro_notch2[i], bank->sample_rate_hz, bank->notch2_hz, 4.0f);
        biquad_init_notch(&bank->gyro_notch_fft[i], bank->sample_rate_hz, bank->fft_peak_hz, 3.5f);
    }
}

void dsp_filter_bank_update_rpm_notch(dsp_filter_bank_t *bank, float esc_fundamental_hz)
{
    if (bank == NULL || esc_fundamental_hz < 40.0f) {
        return;
    }
    if (fabsf(esc_fundamental_hz - bank->notch1_hz) > 4.0f) {
        bank->notch1_hz = esc_fundamental_hz;
        bank->notch2_hz = esc_fundamental_hz * 2.0f;
        for (size_t i = 0U; i < 3U; i++) {
            biquad_update_notch_coeffs(&bank->gyro_notch1[i], bank->sample_rate_hz, bank->notch1_hz, 4.0f);
            biquad_update_notch_coeffs(&bank->gyro_notch2[i], bank->sample_rate_hz, bank->notch2_hz, 4.0f);
        }
    }
}

void dsp_filter_bank_process(dsp_filter_bank_t *bank,
                             const float gyro_in[3],
                             const float accel_in[3],
                             float gyro_out[3],
                             float accel_out[3])
{
    if (bank == NULL) {
        return;
    }

    float g_mag2 = 0.0f;
    for (size_t i = 0U; i < 3U; i++) {
        float g = biquad_apply(&bank->gyro_notch1[i], gyro_in[i]);
        g = biquad_apply(&bank->gyro_notch2[i], g);
        g = biquad_apply(&bank->gyro_notch_fft[i], g);
        gyro_out[i] = biquad_apply(&bank->gyro_lpf[i], g);
        accel_out[i] = biquad_apply(&bank->accel_lpf[i], accel_in[i]);
        g_mag2 += gyro_in[i] * gyro_in[i];
    }

    float g_rms = sqrtf(g_mag2 / 3.0f);
    bank->gyro_rms_rad_s = 0.95f * bank->gyro_rms_rad_s + 0.05f * g_rms;

    /* Feed raw gyro Z into 64-point FFT window */
    bank->fft_ring[bank->fft_idx++] = gyro_in[2];
    if (bank->fft_idx >= DSP_FFT_N) {
        bank->fft_idx = 0U;
        float peak = compute_fft64_peak_hz(bank->fft_ring, bank->sample_rate_hz);
        if (fabsf(peak - bank->fft_peak_hz) > 5.0f) {
            bank->fft_peak_hz = 0.7f * bank->fft_peak_hz + 0.3f * peak;
            for (size_t i = 0U; i < 3U; i++) {
                biquad_update_notch_coeffs(&bank->gyro_notch_fft[i], bank->sample_rate_hz, bank->fft_peak_hz, 3.5f);
            }
        }
    }
}

