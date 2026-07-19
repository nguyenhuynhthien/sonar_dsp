#include "CombineReceiverApp.h"
#include <Constant.hpp>
#include <math.h>
#include <esp_dsp.h>

CombineReceiverApp::CombineReceiverApp(SharedSonarData& sharedData)
    : _sharedData(sharedData) {}

void CombineReceiverApp::begin() {
    // Initialize esp-dsp Radix-2 FFT library for 16-point complex float calculations
    esp_err_t err = dsps_fft2r_init_fc32(NULL, Constant::DOPPLER_FFT_LEN);
    if (err != ESP_OK) {
        Serial.println("Error: Failed to initialize esp-dsp FFT!");
    }
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

extern int16_t s_tempOutI[2][Constant::ADC_SAMPLES];
extern int16_t s_tempOutQ[2][Constant::ADC_SAMPLES];

void CombineReceiverApp::sendSumWaveformFrame(int c) {
    // We use the full Pulse 0 matched filter buffers s_tempOutI/Q
    static int16_t sumBuffer[Constant::ADC_SAMPLES];
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
        int16_t lI = s_tempOutI[0][n];
        int16_t lQ = s_tempOutQ[0][n];
        int16_t rI = s_tempOutI[1][n];
        int16_t rQ = s_tempOutQ[1][n];
        int32_t sqL = (int32_t)lI * lI + (int32_t)lQ * lQ;
        int32_t sqR = (int32_t)rI * rI + (int32_t)rQ * rQ;
        sumBuffer[n] = (int16_t)constrain(isqrt32((sqL + sqR) >> 4), 0, Constant::Q15_MAX);
    }

    if (_com != nullptr) {
        static uint16_t sumWaveFrameId = 0;
        _com->sendFrameAsync(sumWaveFrameId++, sumBuffer, Constant::ADC_SAMPLES, 0); // receiverId = 0 (Sum Channel)
    }
}

void CombineReceiverApp::processTargetAndVelocity() {
    bool targetDetected = false;
    int bestIdx = -1;
    int32_t maxAcc = 0;
    int32_t velocity_bin = 0;
    int32_t cleanRawMag = 0;

    int center = 0;
    taskENTER_CRITICAL(&_sharedData.spinlock);
    center = _sharedData.sharedWindowCenterIdx;
    taskEXIT_CRITICAL(&_sharedData.spinlock);

    int start = center - (Constant::FFT_WINDOW_SIZE / 2);
    int bestK = -1;

    // Find peak range bin by accumulating Sum Channel energy only within the 15-sample window
    for (int k = 0; k < Constant::FFT_WINDOW_SIZE; ++k) {
        int32_t sumEnergy = 0;
        for (int p = 0; p < 8; ++p) {
            int16_t lI = _sharedData.channelL_I[p][k];
            int16_t lQ = _sharedData.channelL_Q[p][k];
            int16_t rI = _sharedData.channelR_I[p][k];
            int16_t rQ = _sharedData.channelR_Q[p][k];
            int32_t energyL = ((int32_t)lI * lI + (int32_t)lQ * lQ) >> 4;
            int32_t energyR = ((int32_t)rI * rI + (int32_t)rQ * rQ) >> 4;
            sumEnergy += energyL + energyR;
        }
        if (sumEnergy > maxAcc) {
            maxAcc = sumEnergy;
            bestK = k;
        }
    }
    
    if (bestK != -1) {
        bestIdx = start + bestK;
    }
    _sharedData.sharedPeakIdx = bestIdx;

    // Threshold detection
    int filterLen = (_sharedData.txPulseLen == (size_t)Constant::BARKER13_PULSE_LEN) ? 
                    (int)Constant::BARKER13_PULSE_LEN : (int)Constant::FILTER_COEFFS_LEN;

    if (bestIdx != -1) {
        int32_t dspGain = (filterLen == (int)Constant::BARKER13_PULSE_LEN) ? 
                          ((int32_t)(Constant::BARKER13_PULSE_LEN / 2) * Constant::DAC_DC_BIAS) : 
                          ((int32_t)(Constant::FILTER_COEFFS_LEN / 2) * Constant::DAC_DC_BIAS);
        int32_t baseThreshold = (filterLen == (int)Constant::BARKER13_PULSE_LEN) ? Constant::BASE_BARKER13_THRESHOLD : Constant::BASE_SINGLE_PULSE_THRESHOLD;
        int32_t localThreshold = baseThreshold << (14 - Constant::MATCHED_FILTER_SHIFT);
        int64_t threshVal = (int64_t)localThreshold * dspGain;
        
        int64_t estimatedMaxValSq = ((int64_t)maxAcc << 4) / 8;
        if (estimatedMaxValSq * Constant::TARGET_THRESHOLD_SCALE_SQ >= threshVal * threshVal) {
            targetDetected = true;
        }
    }

    if (targetDetected && bestK != -1) {
        // Collect Sum Channel samples at the peak index for all 8 pulses
        for (int p = 0; p < 8; ++p) {
            _sharedData.sharedFftReal[p] = (_sharedData.channelL_I[p][bestK] + _sharedData.channelR_I[p][bestK]) >> 1;
            _sharedData.sharedFftImag[p] = (_sharedData.channelL_Q[p][bestK] + _sharedData.channelR_Q[p][bestK]) >> 1;
        }

        // Run 16-point FFT using esp-dsp (8 data points + 8 zero padding)
        // Complex float input/output: [Real, Imag, Real, Imag...] size = 16 * 2 = 32
        static float fftBuffer[Constant::DOPPLER_FFT_LEN * 2] __attribute__((aligned(16)));
        for (int i = 0; i < 8; ++i) {
            fftBuffer[i * 2 + 0] = (float)_sharedData.sharedFftReal[i];
            fftBuffer[i * 2 + 1] = (float)_sharedData.sharedFftImag[i];
        }
        // Zero-padding from index 8 to 15
        for (int i = 8; i < (int)Constant::DOPPLER_FFT_LEN; ++i) {
            fftBuffer[i * 2 + 0] = 0.0f;
            fftBuffer[i * 2 + 1] = 0.0f;
        }

        // Run complex FFT
        dsps_fft2r_fc32(fftBuffer, Constant::DOPPLER_FFT_LEN);
        // Bit reverse to order output frequency bins
        dsps_bit_rev_fc32(fftBuffer, Constant::DOPPLER_FFT_LEN);

        // Find peak frequency bin
        velocity_bin = 0;
        float maxFftMagSq = -1.0f;
        for (int k = 0; k < (int)Constant::DOPPLER_FFT_LEN; ++k) {
            float re = fftBuffer[k * 2 + 0];
            float im = fftBuffer[k * 2 + 1];
            float magSq = re * re + im * im;
            if (magSq > maxFftMagSq) {
                maxFftMagSq = magSq;
                velocity_bin = k;
            }
        }

        if (velocity_bin >= (int)Constant::DOPPLER_FFT_LEN / 2) {
            velocity_bin -= (int)Constant::DOPPLER_FFT_LEN;
        }

        cleanRawMag = (int32_t)sqrtf(maxFftMagSq);
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
            _com->sendTarget(bestIdx, angleToSend, cleanRawMag, velocity_bin, 0); // receiverId = 0 (Sum Channel)
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
        // Prepare for the next pulse
        taskENTER_CRITICAL(&_sharedData.spinlock);
        _sharedData.pulseIndex = nextPulseIdx;
        taskEXIT_CRITICAL(&_sharedData.spinlock);
    } else {
        // 8th pulse processed. Send Sum Channel waveform (Rx0) using Pulse 0 and process target details.
        sendSumWaveformFrame(0);
        processTargetAndVelocity();

        // Reset for the next batch of 8 pulses
        taskENTER_CRITICAL(&_sharedData.spinlock);
        _sharedData.pulseIndex = 0;
        _sharedData.accumulatedDataReady = true;
        _sharedData.requestServoStep = true;
        _sharedData.stepComplete = true;
        taskEXIT_CRITICAL(&_sharedData.spinlock);
    }
}
