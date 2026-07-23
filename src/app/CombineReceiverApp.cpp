#include "CombineReceiverApp.h"
#include <Constant.hpp>
#include <math.h>
#include <esp_dsp.h>
#include "../service/XtensaDspSimdHelper.h"

CombineReceiverApp::CombineReceiverApp(SharedSonarData& sharedData)
    : _sharedData(sharedData) {}

void CombineReceiverApp::begin() {
}

// Fast integer square root helper
static uint32_t isqrt32(uint32_t n) {
    uint32_t res = 0;
    uint32_t bit = 1u << 30;
    while (bit > n) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (n >= res + bit) {
            n -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

void CombineReceiverApp::sendSumWaveformFrame(int c) {
    if (_com != nullptr) {
        static uint16_t sumWaveFrameId = 0;
        _com->sendFrameAsync(sumWaveFrameId++, _sharedData.sumAccumulator, Constant::ADC_SAMPLES, 0); // receiverId = 0 (Sum Channel)
    }
}

void CombineReceiverApp::processTargetAndVelocity() {
    bool targetDetected = false;
    int bestIdx = -1;
    int32_t maxAcc = 0;
    int32_t velocity_bin = 0;
    float velocity_bin_frac = 0.0f;
    int32_t cleanRawMag = 0;

    // 1. Compute 8-pulse accumulated Sum magnitude across all 2048 samples (scaled >> 1 to fit int16_t Q15 range without peak clipping)
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
        int32_t totalMag = 0;
        for (int p = 0; p < 8; ++p) {
            int32_t re = _sharedData.matrixSum_I[p][n];
            int32_t im = _sharedData.matrixSum_Q[p][n];
            int32_t sqVal = (re * re + im * im) >> 4;
            totalMag += isqrt32(sqVal);
        }
        _sharedData.sumAccumulator[n] = (int16_t)constrain(totalMag >> 1, 0, Constant::Q15_MAX);
    }

    // 2. Find peak index across all 2048 samples (ignoring TX blanking region)
    for (int n = Constant::TX_LEAKAGE_BLANK_SAMPLES; n < (int)Constant::ADC_SAMPLES; ++n) {
        int32_t mag = _sharedData.sumAccumulator[n];
        if (mag > maxAcc) {
            maxAcc = mag;
            bestIdx = n;
        }
    }
    _sharedData.sharedPeakIdx = bestIdx;

    // Target threshold detection based on Q15 magnitude scale
    int filterLen = (_sharedData.txPulseLen == (size_t)Constant::BARKER13_PULSE_LEN) ? 
                    (int)Constant::BARKER13_PULSE_LEN : (int)Constant::FILTER_COEFFS_LEN;

    // Dynamic CFAR noise-floor threshold to prevent false target detections
    if (bestIdx != -1) {
        int32_t noiseSum = 0;
        int noiseCount = 0;
        for (int n = Constant::TX_LEAKAGE_BLANK_SAMPLES; n < (int)Constant::ADC_SAMPLES; ++n) {
            noiseSum += _sharedData.sumAccumulator[n];
            noiseCount++;
        }
        int32_t noiseMean = (noiseCount > 0) ? (noiseSum / noiseCount) : 100;

        int32_t minFloor = (filterLen == (int)Constant::BARKER13_PULSE_LEN) ? 700 : 900;
        int32_t dynamicThresh = noiseMean * 3;
        if (dynamicThresh < minFloor) dynamicThresh = minFloor;

        if (maxAcc >= dynamicThresh) {
            targetDetected = true;
        }
    }

    if (targetDetected && bestIdx != -1) {
        float maxFftMagSq = -1.0f;
        int peakRangeBin = bestIdx;
        velocity_bin = 0;

        int windowStart = bestIdx - (Constant::FFT_WINDOW_SIZE / 2);
        ComplexFloat fftIn[16];
        ComplexFloat fftOut[16];

        // Perform 16-point Doppler FFT (with 8-zero padding) across all 15 range bins centered around bestIdx
        for (int k = 0; k < Constant::FFT_WINDOW_SIZE; ++k) {
            int rangeBin = windowStart + k;
            if (rangeBin < 0 || rangeBin >= (int)Constant::ADC_SAMPLES) continue;

            // Load 8 complex samples and pad 8 zeros
            for (int p = 0; p < 8; ++p) {
                fftIn[p].re = (float)_sharedData.matrixSum_I[p][rangeBin];
                fftIn[p].im = (float)_sharedData.matrixSum_Q[p][rangeBin];
            }
            for (int p = 8; p < 16; ++p) {
                fftIn[p].re = 0.0f;
                fftIn[p].im = 0.0f;
            }

            // Run 16-point unrolled float FFT
            fft16_float_unrolled(fftIn, fftOut);

            // Search peak Doppler frequency bin across this range bin (length 16)
            for (int f = 0; f < 16; ++f) {
                float re = fftOut[f].re;
                float im = fftOut[f].im;
                float magSq = re * re + im * im;
                if (magSq > maxFftMagSq) {
                    maxFftMagSq = magSq;
                    velocity_bin = f;
                    peakRangeBin = rangeBin;
                }
            }
        }

        // Re-run FFT at the peak range bin to perform quadratic peak interpolation
        for (int p = 0; p < 8; ++p) {
            fftIn[p].re = (float)_sharedData.matrixSum_I[p][peakRangeBin];
            fftIn[p].im = (float)_sharedData.matrixSum_Q[p][peakRangeBin];
        }
        for (int p = 8; p < 16; ++p) {
            fftIn[p].re = 0.0f;
            fftIn[p].im = 0.0f;
        }
        fft16_float_unrolled(fftIn, fftOut);

        int f_peak = velocity_bin;
        int f_prev = (f_peak - 1 + 16) % 16;
        int f_next = (f_peak + 1) % 16;
        float y0 = sqrtf(fftOut[f_prev].re * fftOut[f_prev].re + fftOut[f_prev].im * fftOut[f_prev].im);
        float y1 = sqrtf(fftOut[f_peak].re * fftOut[f_peak].re + fftOut[f_peak].im * fftOut[f_peak].im);
        float y2 = sqrtf(fftOut[f_next].re * fftOut[f_next].re + fftOut[f_next].im * fftOut[f_next].im);

        int bin_centered = velocity_bin;
        if (bin_centered >= 8) {
            bin_centered -= 16;
        }

        float d = 0.0f;
        float denominator = 2.0f * y1 - y0 - y2;
        if (denominator > 1e-3f) {
            d = 0.5f * (y2 - y0) / denominator;
            if (d < -0.5f) d = -0.5f;
            if (d > 0.5f) d = 0.5f;
        }
        velocity_bin_frac = (float)bin_centered + d;

        cleanRawMag = (int32_t)sqrtf(maxFftMagSq);
        bestIdx = peakRangeBin;
    }

    // Send target details to SonarViewer
    if (_com != nullptr) {
        uint16_t currentAngle = 0;
        bool isCCW = true;
        taskENTER_CRITICAL(&_sharedData.spinlock);
        currentAngle = _sharedData.servoAngle;
        isCCW = _sharedData.sweepDirectionCCW;
        taskEXIT_CRITICAL(&_sharedData.spinlock);

        if (targetDetected && bestIdx != -1) {
            uint16_t angleToSend = currentAngle;
            if (!isCCW) angleToSend |= 0x8000;

            // 1. Convert range bin to meters
            int tof_idx = bestIdx - filterLen;
            if (tof_idx < 0) tof_idx = 0;
            float t_range = ((float)tof_idx * Constant::SPEED_OF_SOUND) / (Constant::ROUND_TRIP_FACTOR * (float)Constant::SAMPLE_RATE);

            // 2. Scale Q15 amplitude and convert to dB
            float raw_strength = (float)cleanRawMag * Constant::GAIN_RESTORATION_FACTOR;
            if (raw_strength < Constant::MIN_AMPLITUDE_LIMIT) raw_strength = Constant::MIN_AMPLITUDE_LIMIT;
            float t_strength = Constant::DB_SCALE_FACTOR * log10f(raw_strength / (float)Constant::ADC_RESOLUTION_MAX * Constant::ADC_REF_VOLTS);

            // 3. Convert doppler bin to velocity
            float pri_seconds = (float)_sharedData.txPeriodMs / 1000.0f;
            float delta_f = 1.0f / (16.0f * pri_seconds);
            float fd = velocity_bin_frac * delta_f;
            float t_velocity = -fd * Constant::SPEED_OF_SOUND / (Constant::ROUND_TRIP_FACTOR * (float)Constant::CENTER_FREQ);

            _com->sendTarget(t_range, angleToSend, t_strength, t_velocity, 0); // receiverId = 0 (Sum Channel)
        }
    }
}

void CombineReceiverApp::run() {
    // Only execute if both ReceiverApp channels have processed the current pulse
    uint8_t doneMask = 0;
    taskENTER_CRITICAL(&_sharedData.spinlock);
    doneMask = _sharedData.processingDone;
    taskEXIT_CRITICAL(&_sharedData.spinlock);

    if (doneMask != 3) {
        return; // Wait for both channels
    }

    int c = _sharedData.pulseIndex;
    int nextPulseIdx = c + 1;

    if (nextPulseIdx < 8) {
        // Pulses 0 to 6: Advance pulse index within the 8-pulse CPI burst
        taskENTER_CRITICAL(&_sharedData.spinlock);
        _sharedData.pulseIndex = nextPulseIdx;
        taskEXIT_CRITICAL(&_sharedData.spinlock);
    } else {
        // Pulse 7: 8th pulse finished. Process 8-pulse CPI accumulation, Doppler FFT, target details, and send Rx0 frame
        processTargetAndVelocity();
        sendSumWaveformFrame(7);

        // Signal full 8-pulse step completion to TransmitterApp and trigger Servo step
        taskENTER_CRITICAL(&_sharedData.spinlock);
        _sharedData.pulseIndex = 0;
        _sharedData.accumulatedDataReady = true;
        _sharedData.requestServoStep = true;
        _sharedData.stepComplete = true;
        taskEXIT_CRITICAL(&_sharedData.spinlock);
    }
}
