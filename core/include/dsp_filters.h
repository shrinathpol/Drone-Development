#ifndef DSP_FILTERS_H
#define DSP_FILTERS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DSP_FFT_N 64U

typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquad_filter_t;

typedef struct {
    float sample_rate_hz;
    biquad_filter_t gyro_lpf[3];
    biquad_filter_t accel_lpf[3];
    biquad_filter_t gyro_notch1[3]; /* Fundamental ESC RPM notch */
    biquad_filter_t gyro_notch2[3]; /* 2nd harmonic notch */
    biquad_filter_t gyro_notch_fft[3]; /* Dynamic FFT peak notch */

    float notch1_hz;
    float notch2_hz;
    float fft_peak_hz;
    float gyro_rms_rad_s;

    float fft_ring[DSP_FFT_N];
    size_t fft_idx;
    uint16_t fft_decim;
} dsp_filter_bank_t;

void biquad_init_lpf(biquad_filter_t *f, float sample_hz, float cutoff_hz);
void biquad_init_notch(biquad_filter_t *f, float sample_hz, float center_hz, float q_factor);
void biquad_update_notch_coeffs(biquad_filter_t *f, float sample_hz, float center_hz, float q_factor);
float biquad_apply(biquad_filter_t *f, float x);

void dsp_filter_bank_init(dsp_filter_bank_t *bank,
                          float sample_rate_hz,
                          float gyro_lpf_hz,
                          float accel_lpf_hz);

void dsp_filter_bank_update_rpm_notch(dsp_filter_bank_t *bank, float esc_fundamental_hz);

void dsp_filter_bank_process(dsp_filter_bank_t *bank,
                             const float gyro_in[3],
                             const float accel_in[3],
                             float gyro_out[3],
                             float accel_out[3]);

#ifdef __cplusplus
}
#endif

#endif /* DSP_FILTERS_H */

