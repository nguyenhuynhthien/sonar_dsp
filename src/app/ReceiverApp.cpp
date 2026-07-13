#include "ReceiverApp.hpp"
#include "../service/AdcService.h"
#include "../service/DacService.h"
#include "../service/SyncSignalService.h"
#include <math.h>
#include <esp_dsp.h>

ReceiverApp::ReceiverApp(SharedSonarData &sharedData)
    : _sharedData(sharedData), _pingCounter(0) {}

void ReceiverApp::begin() {
  // Allocate DSP buffers on the heap (approx 32KB total, which fits easily)
  _demodI = new (std::nothrow) float[Constant::ADC_SAMPLES];
  _demodQ = new (std::nothrow) float[Constant::ADC_SAMPLES];
  _filteredI = new (std::nothrow) float[Constant::ADC_SAMPLES];
  _filteredQ = new (std::nothrow) float[Constant::ADC_SAMPLES];

  for (int i = 0; i < 8; ++i) {
    _pulseHistory[i] = new (std::nothrow) float[Constant::ADC_SAMPLES];
  }
  _accumulatedRaw = new (std::nothrow) float[Constant::ADC_SAMPLES];

  bool allocSuccess = _demodI && _demodQ && _filteredI && _filteredQ && _accumulatedRaw;
  for (int i = 0; i < 8; ++i) {
    if (!_pulseHistory[i]) allocSuccess = false;
  }

  if (!allocSuccess) {
    Serial.println("Error: Failed to allocate DSP/accumulation buffers on heap!");
  }

  // Initialize ESP-DSP Radix-2 FFT factors
  esp_err_t err = dsps_fft2r_init_fc32(NULL, Constant::SLOW_TIME_LEN);
  if (err != ESP_OK) {
    Serial.printf("Error: ESP-DSP FFT initialization failed (code %d)\n", err);
  }

  initDSPCoefficients();
}

void ReceiverApp::initDSPCoefficients() {
  uint8_t localTxBuffer[Constant::BARKER13_PULSE_LEN];
  int pulseLen;

  taskENTER_CRITICAL(&_sharedData.spinlock);
  pulseLen = _sharedData.txPulseLen;
  memcpy(localTxBuffer, _sharedData.txBuffer, pulseLen);
  taskEXIT_CRITICAL(&_sharedData.spinlock);

  _filterLen = pulseLen;

  // Matched filter is time-reversed conjugate of demodulated TX pulse
  for (int i = 0; i < pulseLen; ++i) {
    float x = (float)localTxBuffer[i] -
              Constant::ADC_DC_OFFSET / 16.0f; // Scale reference to match input

    float refCos = 0.0f;
    float refSin = 0.0f;
    switch (i & 3) {
    case 0:
      refCos = 1.0f;
      refSin = 0.0f;
      break;
    case 1:
      refCos = 0.0f;
      refSin = 1.0f;
      break;
    case 2:
      refCos = -1.0f;
      refSin = 0.0f;
      break;
    case 3:
      refCos = 0.0f;
      refSin = -1.0f;
      break;
    }

    float demodI = x * refCos;
    float demodQ = x * (-refSin);

    _hI[(pulseLen - 1) - i] = demodI;
    _hQ[(pulseLen - 1) - i] = -demodQ; // Conjugate
  }
}

void ReceiverApp::performIQDemodulation(const float *rawSamples) {
  if (!_demodI || !_demodQ)
    return;

  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    float x = rawSamples[n];

    // Optimized LO cos/sin are [1, 0, -1, 0] and [0, 1, 0, -1] respectively.
    // Q = x * (-refSin) -> refSin is [0, 1, 0, -1] -> -refSin is [0, -1, 0, 1].
    switch (n & 3) {
    case 0:
      _demodI[n] = x;
      _demodQ[n] = 0.0f;
      break;
    case 1:
      _demodI[n] = 0.0f;
      _demodQ[n] = -x;
      break;
    case 2:
      _demodI[n] = -x;
      _demodQ[n] = 0.0f;
      break;
    case 3:
      _demodI[n] = 0.0f;
      _demodQ[n] = x;
      break;
    }
  }
}

void ReceiverApp::performMatchedFiltering() {
  if (!_demodI || !_demodQ || !_filteredI || !_filteredQ)
    return;
  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    float sumI = 0.0f;
    float sumQ = 0.0f;

    // Convolve the complex signal with complex filter coefficients
    // Optimization: since LO has periodic 0s, hI/hQ also have periodic 0s.
    // We only perform calculations for non-zero coefficients.
    for (int k = 0; k < _filterLen; ++k) {
      if (n >= k) {
        float sigI = _demodI[n - k];
        float sigQ = _demodQ[n - k];
        float coefI = _hI[k];
        float coefQ = _hQ[k];

        if (coefI != 0.0f) {
          sumI += sigI * coefI;
          sumQ += sigQ * coefI;
        }
        if (coefQ != 0.0f) {
          sumI -= sigQ * coefQ;
          sumQ += sigI * coefQ;
        }
      }
    }
    _filteredI[n] = sumI;
    _filteredQ[n] = sumQ;
  }
}

void ReceiverApp::fftRadix2(float *real, float *imag, int n) {
  // Pack real/imag arrays into interleaved complex array
  float fft_input[2 * Constant::SLOW_TIME_LEN];
  for (int i = 0; i < n; ++i) {
    fft_input[2 * i] = real[i];
    fft_input[2 * i + 1] = imag[i];
  }

  // Perform Bit Reversal and Radix-2 FFT using hardware-accelerated ESP-DSP library
  dsps_bit_rev_fc32(fft_input, n);
  dsps_fft2r_fc32(fft_input, n);

  // Unpack back to real and imag arrays
  for (int i = 0; i < n; ++i) {
    real[i] = fft_input[2 * i];
    imag[i] = fft_input[2 * i + 1];
  }
}

void ReceiverApp::performPulseDopplerFFT() {
  if (!_filteredI || !_filteredQ || !_slowTimeI[0] || !_slowTimeQ[0])
    return;
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
      // Re-order buffer starting from current write head so it is sequential in
      // time
      int histIdx = (_pingCounter + i) % (int)Constant::SLOW_TIME_LEN;
      real[i] = _slowTimeI[histIdx][n];
      imag[i] = _slowTimeQ[histIdx][n];
    }

    fftRadix2(real, imag, (int)Constant::SLOW_TIME_LEN);
  }
}

void ReceiverApp::run() {
  // Dynamically update matched filter coefficients if pulse type changed
  initDSPCoefficients();

  // Capture the raw samples under lock
  static uint16_t localRawSamples[Constant::ADC_SAMPLES];
  taskENTER_CRITICAL(&_sharedData.spinlock);
  memcpy(localRawSamples, (const void *)_sharedData.adcBuffer,
         sizeof(localRawSamples));
  taskEXIT_CRITICAL(&_sharedData.spinlock);

  // 1. Digital DC Filter: compute the exact mean of the 2048 samples
  float sum = 0.0f;
  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    sum += localRawSamples[n];
  }
  float mean = sum / (float)Constant::ADC_SAMPLES;

  // 2. Save DC-filtered pulse to _pulseHistory
  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    _pulseHistory[_accumulatedCount][n] = (float)localRawSamples[n] - mean;
  }
  _accumulatedCount++;

  if (_accumulatedCount < 8) {
    // Not enough pulses accumulated yet, signal Core 0 to continue firing
    taskENTER_CRITICAL(&_sharedData.spinlock);
    _sharedData.processingDone = true;
    taskEXIT_CRITICAL(&_sharedData.spinlock);
    return;
  }

  // 3. We have integrated 8 pulses. Sum them up to form _accumulatedRaw.
  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    _accumulatedRaw[n] = _pulseHistory[0][n];
  }
  for (int p = 1; p < 8; ++p) {
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
      _accumulatedRaw[n] += _pulseHistory[p][n];
    }
  }

  // 3. We have integrated 8 pulses. Do not divide by 8 (UI will handle the division).
  // Run DSP algorithms on the accumulated raw sum
  performIQDemodulation(_accumulatedRaw);
  performMatchedFiltering();
  performPulseDopplerFFT();

  // Calculate magnitude, peak detection for target range and strength
  static float magnitude[Constant::ADC_SAMPLES];
  float maxMag = 0.0f;
  int peakIdx = -1;

  // Normalization factor (unmodified, but maxMag will be 8x larger because we didn't divide by 8)
  float scaleFactor = (_filterLen == (int)Constant::BARKER13_PULSE_LEN)
                          ? (sqrtf(13.0f) / (52.0f * 127.0f))
                          : (1.0f / (16.0f * 127.0f));

  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    float rawMag =
        sqrtf(_filteredI[n] * _filteredI[n] + _filteredQ[n] * _filteredQ[n]);
    float mag = rawMag * scaleFactor;
    magnitude[n] = mag;

    // Skip the first 120 samples to avoid TX pulse leakage
    if (n >= 120 && mag > maxMag) {
      maxMag = mag;
      peakIdx = n;
    }
  }

  // Local threshold is scaled by 8.0f since maxMag is 8x larger (adjusted to 200.0f for better sensitivity)
  float localThreshold = 200.0f * 8.0f;
  if (peakIdx != -1 && maxMag > localThreshold) {
    int tofIdx = peakIdx - _filterLen;
    if (tofIdx < 0)
      tofIdx = 0;

    float range = ((float)tofIdx * Constant::SPEED_OF_SOUND) /
                  (2.0f * (float)Constant::SAMPLE_RATE);

    taskENTER_CRITICAL(&_sharedData.spinlock);
    _sharedData.targetDetected = true;
    _sharedData.targetRange = range;
    _sharedData.targetStrength = maxMag;
    taskEXIT_CRITICAL(&_sharedData.spinlock);
  } else {
    taskENTER_CRITICAL(&_sharedData.spinlock);
    _sharedData.targetDetected = false;
    taskEXIT_CRITICAL(&_sharedData.spinlock);
  }

  // Overwrite adcBuffer based on the selected streaming mode
  taskENTER_CRITICAL(&_sharedData.spinlock);
  uint8_t mode = _sharedData.streamMode;
  if (mode == 0) { // STREAM_RAW
    // Since we accumulated 8 pulses, shift the DC-offset by Constant::ADC_DC_OFFSET * 8.0f (16384.0f)
    // to prevent going negative, then constrain to uint16_t range [0.0f, 65535.0f].
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
      float biasedRaw = _accumulatedRaw[n] + (Constant::ADC_DC_OFFSET * 8.0f);
      _sharedData.adcBuffer[n] = (uint16_t)constrain(biasedRaw, 0.0f, 65535.0f);
    }
  } else if (mode == 1) { // STREAM_DEMOD
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
      float demodMag = sqrtf(_demodI[n] * _demodI[n] + _demodQ[n] * _demodQ[n]);
      _sharedData.adcBuffer[n] = (uint16_t)constrain(demodMag, 0.0f, 65535.0f);
    }
  } else if (mode == 2) { // STREAM_COMPRESSED
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
      _sharedData.adcBuffer[n] = (uint16_t)constrain(magnitude[n], 0.0f, 65535.0f);
    }
  }

  _sharedData.accumulatedDataReady = true;
  _sharedData.processingDone = true;
  taskEXIT_CRITICAL(&_sharedData.spinlock);

  // Reset accumulation for the next cycle
  _accumulatedCount = 0;
}
