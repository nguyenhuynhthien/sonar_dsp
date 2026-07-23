#include "XtensaDspSimdHelper.h"
#include <Constant.hpp>

#if !__has_include(<xtensa/tie/xt_DSP.h>)
ae_int16x2 AE_MOVDA16(int16_t h, int16_t l) {
    ae_int16x2 r;
    r.d[0] = l;
    r.d[1] = h;
    return r;
}

ae_int16x2 AE_ADD16(ae_int16x2 a, ae_int16x2 b) {
    ae_int16x2 r;
    r.d[0] = a.d[0] + b.d[0];
    r.d[1] = a.d[1] + b.d[1];
    return r;
}

ae_int16x2 AE_SUB16(ae_int16x2 a, ae_int16x2 b) {
    ae_int16x2 r;
    r.d[0] = a.d[0] - b.d[0];
    r.d[1] = a.d[1] - b.d[1];
    return r;
}
#endif

// Custom Q15 convolution implementation
esp_err_t dsps_conv_q15(const int16_t *Signal, const int siglen, const int16_t *Kernel, const int kernlen, int16_t *convout) {
    int out_len = siglen + kernlen - 1;
    for (int n = 0; n < out_len; ++n) {
        int32_t sum = 0;
        int k_start = (n >= siglen) ? (n - siglen + 1) : 0;
        int k_end = (n < kernlen) ? n : (kernlen - 1);
        for (int k = k_start; k <= k_end; ++k) {
            sum += ((int32_t)Signal[n - k] * Kernel[k] + (1 << (Constant::Q15_SHIFT - 1))) >> Constant::Q15_SHIFT;
        }
        convout[n] = (int16_t)constrain(sum, Constant::Q15_MIN, Constant::Q15_MAX);
    }
    return ESP_OK;
}

// Hàm tính nhanh FFT 8 điểm không dùng vòng lặp (Unrolled) bằng SIMD Xtensa
void fft8_q15_simd(const Complex16* in, Complex16* out) {
    const ae_int16x2* pIn = (const ae_int16x2*)in;
    ae_int16x2* pOut = (ae_int16x2*)out;
    // 1. Nạp dữ liệu theo thứ tự đảo bit (Bit-reversed load)
    ae_int16x2 s0 = pIn[0], s1 = pIn[4], s2 = pIn[2], s3 = pIn[6];
    ae_int16x2 s4 = pIn[1], s5 = pIn[5], s6 = pIn[3], s7 = pIn[7];
    // 2. Tầng bướm 1 (Stage 1 - Cộng/trừ song song hai thành phần thực/ảo)
    ae_int16x2 t0 = AE_ADD16(s0, s1);
    ae_int16x2 t1 = AE_SUB16(s0, s1);
    ae_int16x2 t2 = AE_ADD16(s2, s3);
    ae_int16x2 t3 = AE_SUB16(s2, s3);
    ae_int16x2 t4 = AE_ADD16(s4, s5);
    ae_int16x2 t5 = AE_SUB16(s4, s5);
    ae_int16x2 t6 = AE_ADD16(s6, s7);
    ae_int16x2 t7 = AE_SUB16(s6, s7);
    // 3. Tầng bướm 2 (Stage 2 - Nhân hệ số quay W_4_1 = -j)
    ae_int16x2 u0 = AE_ADD16(t0, t2);
    ae_int16x2 u2 = AE_SUB16(t0, t2);
    int16_t t1_re = AE_MOV2X16_0(t1);
    int16_t t1_im = AE_MOV2X16_1(t1);
    int16_t t3_re = AE_MOV2X16_0(t3);
    int16_t t3_im = AE_MOV2X16_1(t3);
    ae_int16x2 u1 = AE_MOVDA16(t1_im - t3_re, t1_re + t3_im); // t1 - j*t3
    ae_int16x2 u3 = AE_MOVDA16(t1_im + t3_re, t1_re - t3_im); // t1 + j*t3
    ae_int16x2 u4 = AE_ADD16(t4, t6);
    ae_int16x2 u6 = AE_SUB16(t4, t6);
    int16_t t5_re = AE_MOV2X16_0(t5);
    int16_t t5_im = AE_MOV2X16_1(t5);
    int16_t t7_re = AE_MOV2X16_0(t7);
    int16_t t7_im = AE_MOV2X16_1(t7);
    ae_int16x2 u5 = AE_MOVDA16(t5_im - t7_re, t5_re + t7_im); // t5 - j*t7
    ae_int16x2 u7 = AE_MOVDA16(t5_im + t7_re, t5_re - t7_im); // t5 + j*t7
    // 4. Tầng bướm 3 (Stage 3 - Nhân hệ số quay W_8_1 và W_8_3)
    auto q15_mul = [](int16_t val) -> int16_t {
        return (int16_t)(((int32_t)val * Constant::COS_PI_4_Q15) >> Constant::Q15_SHIFT);
    };
    // Tính toán tích lượng phức: W_8_1 * u5 = c*(1-j)*(u5.re + j*u5.im)
    int16_t u5_re = AE_MOV2X16_0(u5);
    int16_t u5_im = AE_MOV2X16_1(u5);
    ae_int16x2 w1_u5 = AE_MOVDA16(
        q15_mul(u5_im - u5_re),
        q15_mul(u5_re + u5_im)
    );
    // Tính toán tích lượng phức: W_8_3 * u7 = -c*(1+j)*(u7.re + j*u7.im)
    int16_t u7_re = AE_MOV2X16_0(u7);
    int16_t u7_im = AE_MOV2X16_1(u7);
    ae_int16x2 w3_u7 = AE_MOVDA16(
        (int16_t)(-q15_mul(u7_re + u7_im)),
        q15_mul(u7_im - u7_re)
    );
    // Ghép tổng hợp lần cuối và ghi kết quả (ghi song song 32-bit thực/ảo)
    pOut[0] = AE_ADD16(u0, u4);
    pOut[1] = AE_ADD16(u1, w1_u5);
    int16_t u2_re = AE_MOV2X16_0(u2);
    int16_t u2_im = AE_MOV2X16_1(u2);
    int16_t u6_re = AE_MOV2X16_0(u6);
    int16_t u6_im = AE_MOV2X16_1(u6);
    pOut[2] = AE_MOVDA16(u2_im - u6_re, u2_re + u6_im); // u2 - j*u6
    pOut[3] = AE_ADD16(u3, w3_u7);
    pOut[4] = AE_SUB16(u0, u4);
    pOut[5] = AE_SUB16(u1, w1_u5);
    pOut[6] = AE_MOVDA16(u2_im + u6_re, u2_re - u6_im); // u2 + j*u6
    pOut[7] = AE_SUB16(u3, w3_u7);
}

// 16-point float FFT, unrolled
void fft16_float_unrolled(const ComplexFloat* in, ComplexFloat* out) {
    // 1. Bit-reversal copy
    out[0]  = in[0];
    out[1]  = in[8];
    out[2]  = in[4];
    out[3]  = in[12];
    out[4]  = in[2];
    out[5]  = in[10];
    out[6]  = in[6];
    out[7]  = in[14];
    out[8]  = in[1];
    out[9]  = in[9];
    out[10] = in[5];
    out[11] = in[13];
    out[12] = in[3];
    out[13] = in[11];
    out[14] = in[7];
    out[15] = in[15];

    // Stage 1: subgroups of size 2 (twiddle factor W_2^0 = 1)
    for (int i = 0; i < 16; i += 2) {
        ComplexFloat temp = out[i + 1];
        out[i + 1].re = out[i].re - temp.re;
        out[i + 1].im = out[i].im - temp.im;
        out[i].re     = out[i].re + temp.re;
        out[i].im     = out[i].im + temp.im;
    }

    // Stage 2: subgroups of size 4 (twiddle factors: W_4^0 = 1, W_4^1 = -j)
    for (int i = 0; i < 16; i += 4) {
        // k = 0
        ComplexFloat temp0 = out[i + 2];
        out[i + 2].re = out[i].re - temp0.re;
        out[i + 2].im = out[i].im - temp0.im;
        out[i].re     = out[i].re + temp0.re;
        out[i].im     = out[i].im + temp0.im;

        // k = 1: multiply out[i+3] by -j
        ComplexFloat temp1 = out[i + 3];
        float t1_re = temp1.im;
        float t1_im = -temp1.re;
        out[i + 3].re = out[i + 1].re - t1_re;
        out[i + 3].im = out[i + 1].im - t1_im;
        out[i + 1].re = out[i + 1].re + t1_re;
        out[i + 1].im = out[i + 1].im + t1_im;
    }

    // Stage 3: subgroups of size 8 (twiddle factors: W_8^k)
    const float C8 = 0.70710678f;
    for (int i = 0; i < 16; i += 8) {
        // k = 0
        ComplexFloat temp0 = out[i + 4];
        out[i + 4].re = out[i].re - temp0.re;
        out[i + 4].im = out[i].im - temp0.im;
        out[i].re     = out[i].re + temp0.re;
        out[i].im     = out[i].im + temp0.im;

        // k = 1: multiply by C8 - j*C8
        ComplexFloat temp1 = out[i + 5];
        float t1_re = C8 * (temp1.re + temp1.im);
        float t1_im = C8 * (temp1.im - temp1.re);
        out[i + 5].re = out[i + 1].re - t1_re;
        out[i + 5].im = out[i + 1].im - t1_im;
        out[i + 1].re = out[i + 1].re + t1_re;
        out[i + 1].im = out[i + 1].im + t1_im;

        // k = 2: multiply by -j
        ComplexFloat temp2 = out[i + 6];
        float t2_re = temp2.im;
        float t2_im = -temp2.re;
        out[i + 6].re = out[i + 2].re - t2_re;
        out[i + 6].im = out[i + 2].im - t2_im;
        out[i + 2].re = out[i + 2].re + t2_re;
        out[i + 2].im = out[i + 2].im + t2_im;

        // k = 3: multiply by -C8 - j*C8
        ComplexFloat temp3 = out[i + 7];
        float t3_re = C8 * (-temp3.re + temp3.im);
        float t3_im = C8 * (-temp3.im - temp3.re);
        out[i + 7].re = out[i + 3].re - t3_re;
        out[i + 7].im = out[i + 3].im - t3_im;
        out[i + 3].re = out[i + 3].re + t3_re;
        out[i + 3].im = out[i + 3].im + t3_im;
    }

    // Stage 4: subgroups of size 16 (twiddle factors: W_16^k)
    const float C16_1_C = 0.92387953f, C16_1_S = 0.38268343f;
    const float C16_3_C = 0.38268343f, C16_3_S = 0.92387953f;

    // k = 0
    {
        ComplexFloat t = out[8];
        out[8].re = out[0].re - t.re;
        out[8].im = out[0].im - t.im;
        out[0].re = out[0].re + t.re;
        out[0].im = out[0].im + t.im;
    }
    // k = 1
    {
        ComplexFloat t = out[9];
        float tr = t.re * C16_1_C + t.im * C16_1_S;
        float ti = t.im * C16_1_C - t.re * C16_1_S;
        out[9].re = out[1].re - tr;
        out[9].im = out[1].im - ti;
        out[1].re = out[1].re + tr;
        out[1].im = out[1].im + ti;
    }
    // k = 2
    {
        ComplexFloat t = out[10];
        float tr = C8 * (t.re + t.im);
        float ti = C8 * (t.im - t.re);
        out[10].re = out[2].re - tr;
        out[10].im = out[2].im - ti;
        out[2].re = out[2].re + tr;
        out[2].im = out[2].im + ti;
    }
    // k = 3
    {
        ComplexFloat t = out[11];
        float tr = t.re * C16_3_C + t.im * C16_3_S;
        float ti = t.im * C16_3_C - t.re * C16_3_S;
        out[11].re = out[3].re - tr;
        out[11].im = out[3].im - ti;
        out[3].re = out[3].re + tr;
        out[3].im = out[3].im + ti;
    }
    // k = 4
    {
        ComplexFloat t = out[12];
        float tr = t.im;
        float ti = -t.re;
        out[12].re = out[4].re - tr;
        out[12].im = out[4].im - ti;
        out[4].re = out[4].re + tr;
        out[4].im = out[4].im + ti;
    }
    // k = 5
    {
        ComplexFloat t = out[13];
        float tr = -t.re * C16_3_C + t.im * C16_3_S;
        float ti = -t.im * C16_3_C - t.re * C16_3_S;
        out[13].re = out[5].re - tr;
        out[13].im = out[5].im - ti;
        out[5].re = out[5].re + tr;
        out[5].im = out[5].im + ti;
    }
    // k = 6
    {
        ComplexFloat t = out[14];
        float tr = C8 * (-t.re + t.im);
        float ti = C8 * (-t.im - t.re);
        out[14].re = out[6].re - tr;
        out[14].im = out[6].im - ti;
        out[6].re = out[6].re + tr;
        out[6].im = out[6].im + ti;
    }
    // k = 7
    {
        ComplexFloat t = out[15];
        float tr = -t.re * C16_1_C + t.im * C16_1_S;
        float ti = -t.im * C16_1_C - t.re * C16_1_S;
        out[15].re = out[7].re - tr;
        out[15].im = out[7].im - ti;
        out[7].re = out[7].re + tr;
        out[7].im = out[7].im + ti;
    }
}

