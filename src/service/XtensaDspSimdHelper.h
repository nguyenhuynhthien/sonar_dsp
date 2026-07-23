#ifndef XTENSA_DSP_SIMD_HELPER_H
#define XTENSA_DSP_SIMD_HELPER_H

#include <stdint.h>
#include <Arduino.h>
#include <esp_err.h>

struct Complex16 {
    int16_t re; // phần thực - ánh xạ vào 16-bit thấp của ae_int16x2
    int16_t im; // phần ảo - ánh xạ vào 16-bit cao của ae_int16x2
};

#if __has_include(<xtensa/tie/xt_DSP.h>)
#include <xtensa/config/core-isa.h>
#include <xtensa/tie/xt_DSP.h>
#else
// Compatibility layer for Xtensa DSP SIMD on standard ESP32 (Xtensa LX6)
typedef union {
    int16_t d[2];
    int32_t v;
} ae_int16x2;

#define AE_MOV2X16_0(r) ((r).d[0])
#define AE_MOV2X16_1(r) ((r).d[1])

ae_int16x2 AE_MOVDA16(int16_t h, int16_t l);
ae_int16x2 AE_ADD16(ae_int16x2 a, ae_int16x2 b);
ae_int16x2 AE_SUB16(ae_int16x2 a, ae_int16x2 b);
#endif

// Reusable function declarations
esp_err_t dsps_conv_q15(const int16_t *Signal, const int siglen, const int16_t *Kernel, const int kernlen, int16_t *convout);
void fft8_q15_simd(const Complex16* in, Complex16* out);

#endif // XTENSA_DSP_SIMD_HELPER_H
