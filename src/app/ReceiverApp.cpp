#include "ReceiverApp.hpp"
#include "../service/AdcService.h"
#include "../service/DacService.h"
#include "../service/SyncSignalService.h"
#include <math.h>

ReceiverApp::ReceiverApp(SharedSonarData& sharedData)
    : _sharedData(sharedData), _pingCounter(0) {
}

void ReceiverApp::begin() {
    initDSPCoefficients();
}

void ReceiverApp::initDSPCoefficients() {
    // Generate coefficients for a matched filter matching a pulse at center frequency
    for (int i = 0; i < (int)Constant::FILTER_COEFFS_LEN; ++i) {
        double angle = (2.0 * M_PI * Constant::CENTER_FREQ * i) / Constant::SAMPLE_RATE;
        // Matched filter coefficients are time-reversed conjugate of transmitted pulse
        _hI[(Constant::FILTER_COEFFS_LEN - 1) - i] = (float)cos(angle);
        _hQ[(Constant::FILTER_COEFFS_LEN - 1) - i] = -(float)sin(angle);
    }
}

void ReceiverApp::performIQDemodulation(const uint16_t* rawSamples) {
    if (!_demodI || !_demodQ) return;
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
        // Local oscillator angle: center frequency.
        double angle = (2.0 * M_PI * Constant::CENTER_FREQ * n) / Constant::SAMPLE_RATE;
        float refCos = (float)cos(angle);
        float refSin = (float)sin(angle);
 
        // Normalize raw samples around DC level
        float x = (float)rawSamples[n] - Constant::ADC_DC_OFFSET;
 
        _demodI[n] = x * refCos;
        _demodQ[n] = x * (-refSin); // conjugate
    }
}

void ReceiverApp::performMatchedFiltering() {
    if (!_demodI || !_demodQ || !_filteredI || !_filteredQ) return;
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
        float sumI = 0.0f;
        float sumQ = 0.0f;
        
        // Convolve the complex signal with complex filter coefficients
        for (int k = 0; k < (int)Constant::FILTER_COEFFS_LEN; ++k) {
            if (n >= k) {
                float sigI = _demodI[n - k];
                float sigQ = _demodQ[n - k];
                float coefI = _hI[k];
                float coefQ = _hQ[k];
                
                sumI += (sigI * coefI - sigQ * coefQ);
                sumQ += (sigI * coefQ + sigQ * coefI);
            }
        }
        _filteredI[n] = sumI;
        _filteredQ[n] = sumQ;
    }
}

void ReceiverApp::fftRadix2(float* real, float* imag, int n) {
    // Bit reversal
    int j = 0;
    for (int i = 0; i < n - 1; ++i) {
        if (i < j) {
            float tempReal = real[i];
            float tempImag = imag[i];
            real[i] = real[j];
            imag[i] = imag[j];
            real[j] = tempReal;
            imag[j] = tempImag;
        }
        int k = n / 2;
        while (k <= j) {
            j -= k;
            k /= 2;
        }
        j += k;
    }

    // Cooley-Tukey Decimation-in-Time
    for (int size = 2; size <= n; size *= 2) {
        int halfSize = size / 2;
        float tabReal = cos(-2.0f * M_PI / size);
        float tabImag = sin(-2.0f * M_PI / size);
        
        for (int i = 0; i < n; i += size) {
            float wReal = 1.0f;
            float wImag = 0.0f;
            for (int k = 0; k < halfSize; ++k) {
                int idx1 = i + k;
                int idx2 = i + k + halfSize;
                
                float tReal = real[idx2] * wReal - imag[idx2] * wImag;
                float tImag = real[idx2] * wImag + imag[idx2] * wReal;
                
                real[idx2] = real[idx1] - tReal;
                imag[idx2] = imag[idx1] - tImag;
                real[idx1] += tReal;
                imag[idx1] += tImag;
                
                float nextWReal = wReal * tabReal - wImag * tabImag;
                wImag = wReal * tabImag + wImag * tabReal;
                wReal = nextWReal;
            }
        }
    }
}

void ReceiverApp::performPulseDopplerFFT() {
    if (!_filteredI || !_filteredQ || !_slowTimeI[0] || !_slowTimeQ[0]) return;
    // 1. Store matched filter output into history buffer at current index
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
        _slowTimeI[_pingCounter][n] = _filteredI[n];
        _slowTimeQ[_pingCounter][n] = _filteredQ[n];
    }
    
    _pingCounter = (_pingCounter + 1) % (int)Constant::SLOW_TIME_LEN;
 
    // 2. Perform Doppler FFT for each range bin
    float real[Constant::SLOW_TIME_LEN];
    float imag[Constant::SLOW_TIME_LEN];
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
        for (int i = 0; i < (int)Constant::SLOW_TIME_LEN; ++i) {
            // Re-order buffer starting from current write head so it is sequential in time
            int histIdx = (_pingCounter + i) % (int)Constant::SLOW_TIME_LEN;
            real[i] = _slowTimeI[histIdx][n];
            imag[i] = _slowTimeQ[histIdx][n];
        }
        
        fftRadix2(real, imag, (int)Constant::SLOW_TIME_LEN);
        
        // The Doppler magnitude can be calculated, but we keep the main flow running 
        // to complete all DSP stages.
    }
}

void ReceiverApp::run() {
    // Capture the raw samples under lock
    static uint16_t localRawSamples[Constant::ADC_SAMPLES];
    taskENTER_CRITICAL(&_sharedData.spinlock);
    memcpy(localRawSamples, (const void*)_sharedData.adcBuffer, sizeof(localRawSamples));
    taskEXIT_CRITICAL(&_sharedData.spinlock);

    // Run DSP algorithms
    performIQDemodulation(localRawSamples);
    performMatchedFiltering();
    performPulseDopplerFFT();
}
