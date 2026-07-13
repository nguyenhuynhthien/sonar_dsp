#include "ReceiverApp.hpp"
#include "../service/ComManager.h"
#include "../service/AdcService.h"
#include "../service/DacService.h"
#include "../service/SyncSignalService.h"
#include <math.h>
#include <esp_dsp.h>

ReceiverApp::ReceiverApp(SharedSonarData &sharedData)
    : _sharedData(sharedData), _pingCounter(0) {}

void ReceiverApp::begin() {
  // Allocate essential DSP buffers only (40KB total)
  _demodI = new (std::nothrow) float[Constant::ADC_SAMPLES];
  _demodQ = new (std::nothrow) float[Constant::ADC_SAMPLES];
  _filteredI = new (std::nothrow) float[Constant::ADC_SAMPLES];
  _filteredQ = new (std::nothrow) float[Constant::ADC_SAMPLES];
  _accumulatedRaw = new (std::nothrow) float[Constant::ADC_SAMPLES];

  bool allocSuccess = _demodI && _demodQ && _filteredI && _filteredQ && _accumulatedRaw;

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

    // matched filter: h(t) = s*(T - t)
    int filterIdx = pulseLen - 1 - i;
    _hI[filterIdx] = x * refCos;
    _hQ[filterIdx] = -x * refSin; // conjugate
  }
}

void ReceiverApp::performIQDemodulation(const float *rawSamples) {
  if (!_demodI || !_demodQ)
    return;

  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    float x = rawSamples[n];
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

  // Real-time matched filtering: convolve I/Q demodulated signal with hI/hQ
  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    float sumI = 0.0f;
    float sumQ = 0.0f;

    for (int k = 0; k < _filterLen; ++k) {
      if (n >= k) {
        float sigI = _demodI[n - k];
        float sigQ = _demodQ[n - k];
        float coefI = _hI[k];
        float coefQ = _hQ[k];

        // Complex multiplication: (sigI + j*sigQ) * (coefI + j*coefQ)
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

  // 2. DC-filter the pulse to a local temp array
  static float tempRaw[Constant::ADC_SAMPLES];
  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    tempRaw[n] = (float)localRawSamples[n] - mean;
  }

  // 3. Process IQ Demodulation and Matched Filtering for the current pulse
  performIQDemodulation(tempRaw);
  performMatchedFiltering();

  float dspGain = (_filterLen == (int)Constant::BARKER13_PULSE_LEN) ? (52.0f * 127.0f) : (16.0f * 127.0f);

  // Step 1: Scan vertically on Column 0 (the very first pulse, accumulatedCount == 0)
  // to find the range peak index r_peak (_peakIdxStored)
  if (_accumulatedCount == 0) {
    float maxRawMag_single = 0.0f;
    _peakIdxStored = -1;

    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
      float rawMag0 = sqrtf(_filteredI[n] * _filteredI[n] + _filteredQ[n] * _filteredQ[n]);
      float normMag0 = rawMag0 / dspGain;

      // Skip the first 120 samples to avoid TX pulse leakage
      if (n >= 120 && normMag0 > maxRawMag_single) {
        maxRawMag_single = normMag0;
        _peakIdxStored = n;
      }
    }

    // Dynamic threshold based on pulse type: Single Pulse needs higher threshold (300.0f) due to higher noise floor
    float localThreshold = (_filterLen == (int)Constant::BARKER13_PULSE_LEN) ? 120.0f : 300.0f;
    if (_peakIdxStored != -1 && maxRawMag_single < localThreshold) {
      _peakIdxStored = -1;
    }

    // Initialize accumulated raw values array for peak detection
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
      _accumulatedRaw[n] = _filteredI[n] * _filteredI[n] + _filteredQ[n] * _filteredQ[n];
    }
  } else {
    // Accumulate incoherent energy for peak validation/detection
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
      _accumulatedRaw[n] += _filteredI[n] * _filteredI[n] + _filteredQ[n] * _filteredQ[n];
    }
  }

  // Extract the complex value of this ping at _peakIdxStored for the horizontal FFT
  if (_peakIdxStored != -1) {
    // Apply a 4-sample moving average (LPF) around the peak to reconstruct a full (I, Q) pair.
    // This ensures both components are non-zero regardless of whether peakIdx is even or odd.
    float sumI = 0.0f;
    float sumQ = 0.0f;
    int count = 0;
    for (int d = -1; d <= 2; ++d) {
      int idx = _peakIdxStored + d;
      if (idx >= 0 && idx < (int)Constant::ADC_SAMPLES) {
        sumI += _filteredI[idx];
        sumQ += _filteredQ[idx];
        count++;
      }
    }
    _fftReal[_accumulatedCount] = sumI / (float)count;
    _fftImag[_accumulatedCount] = sumQ / (float)count;
  } else {
    _fftReal[_accumulatedCount] = 0.0f;
    _fftImag[_accumulatedCount] = 0.0f;
  }

  _accumulatedCount++;

  if (_accumulatedCount < 8) {
    // Not enough pulses accumulated yet, signal Core 0 to continue firing
    taskENTER_CRITICAL(&_sharedData.spinlock);
    _sharedData.pulseIndex = _accumulatedCount;
    _sharedData.processingDone = true;
    taskEXIT_CRITICAL(&_sharedData.spinlock);
    return;
  }

  // We have integrated 8 pulses.
  // Step 2: Quét ngang chạy FFT tại hàng _peakIdxStored
  float velocity = 0.0f;
  float cleanRawMag = 0.0f;
  bool targetDetected = (_peakIdxStored != -1);

  if (targetDetected) {
    // Run 8-point DFT/FFT on the stored ping history at _peakIdxStored
    float realOut[8] = {0.0f};
    float imagOut[8] = {0.0f};
    for (int k = 0; k < 8; ++k) {
      float sumReal = 0.0f;
      float sumImag = 0.0f;
      for (int n = 0; n < 8; ++n) {
        float angle = -2.0f * M_PI * k * n / 8.0f;
        float cosVal = cosf(angle);
        float sinVal = sinf(angle);
        sumReal += _fftReal[n] * cosVal - _fftImag[n] * sinVal;
        sumImag += _fftReal[n] * sinVal + _fftImag[n] * cosVal;
      }
      realOut[k] = sumReal;
      imagOut[k] = sumImag;
    }

    // Find peak frequency bin
    float maxFftMag = -1.0f;
    int doppler_bin = 0;
    for (int k = 0; k < 8; ++k) {
      float fftMag = realOut[k] * realOut[k] + imagOut[k] * imagOut[k];
      if (fftMag > maxFftMag) {
        maxFftMag = fftMag;
        doppler_bin = k;
      }
    }

    // Map doppler_bin to fd
    float fd = 0.0f;
    float deltaF = 1.0f / (8.0f * ((float)Constant::TX_PERIOD_MS / 1000.0f)); // 8.3333 Hz
    if (doppler_bin < 4) {
      fd = doppler_bin * deltaF;
    } else {
      fd = (doppler_bin - 8) * deltaF;
    }

    // Convert to velocity: v = fd * c / (2 * fc)
    velocity = fd * (Constant::SPEED_OF_SOUND / (2.0f * (float)Constant::CENTER_FREQ));

    // Clean unscaled coherent integrated amplitude (8x gain)
    cleanRawMag = sqrtf(maxFftMag);
  }

  // Update target details
  if (targetDetected && _peakIdxStored != -1) {
    int tofIdx = _peakIdxStored - _filterLen;
    if (tofIdx < 0)
      tofIdx = 0;

    float range = ((float)tofIdx * Constant::SPEED_OF_SOUND) /
                  (2.0f * (float)Constant::SAMPLE_RATE);

    taskENTER_CRITICAL(&_sharedData.spinlock);
    _sharedData.targetDetected = true;
    _sharedData.targetRange = range;
    _sharedData.targetStrength = cleanRawMag; // Coherent sum unscaled magnitude (8x gain)
    _sharedData.peakIndexForVelocity = _peakIdxStored;
    _sharedData.velocityRequested = true;
    taskEXIT_CRITICAL(&_sharedData.spinlock);
  } else {
    taskENTER_CRITICAL(&_sharedData.spinlock);
    _sharedData.targetDetected = false;
    _sharedData.velocityRequested = false;
    taskEXIT_CRITICAL(&_sharedData.spinlock);
  }

  // Prepare streaming buffer (STREAM_COMPRESSED maps to incoherent accumulated magnitude of 8 pulses)
  // Normalized to the ADC scale (0-4095)
  float maxAccumulatedMag = 0.0f;
  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    float rawMagAccumulated = sqrtf(_accumulatedRaw[n]);
    float normVal = rawMagAccumulated / dspGain;
    _accumulatedRaw[n] = normVal;
    if (n >= 120 && normVal > maxAccumulatedMag) {
      maxAccumulatedMag = normVal;
    }
  }

  // Copy streaming values to localBuffer
  uint8_t mode = 0;
  taskENTER_CRITICAL(&_sharedData.spinlock);
  mode = _sharedData.streamMode;
  taskEXIT_CRITICAL(&_sharedData.spinlock);

  static uint16_t localBuffer[Constant::ADC_SAMPLES];
  if (mode == 0) { // STREAM_RAW
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
      float biasedRaw = tempRaw[n] + Constant::ADC_DC_OFFSET;
      localBuffer[n] = (uint16_t)constrain(biasedRaw, 0.0f, 65535.0f);
    }
  } else if (mode == 1) { // STREAM_DEMOD
    // Demodulate the last pulse (tempRaw)
    performIQDemodulation(tempRaw);
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
      float demodMag = sqrtf(_demodI[n] * _demodI[n] + _demodQ[n] * _demodQ[n]);
      localBuffer[n] = (uint16_t)constrain(demodMag, 0.0f, 65535.0f);
    }
  } else if (mode == 2) { // STREAM_COMPRESSED
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
      localBuffer[n] = (uint16_t)constrain(_accumulatedRaw[n], 0.0f, 65535.0f);
    }
  }

  // Copy localBuffer to old _sharedData.adcBuffer for general tracking
  memcpy((void*)_sharedData.adcBuffer, localBuffer, sizeof(localBuffer));
  
  taskENTER_CRITICAL(&_sharedData.spinlock);
  bool targetDetectedFinal = _sharedData.targetDetected;
  float targetRangeFinal = _sharedData.targetRange;
  float targetStrengthFinal = _sharedData.targetStrength;
  uint16_t currentAngle = _sharedData.servoAngle;
  
  _sharedData.velocityRequested = false;
  _sharedData.accumulatedDataReady = true;
  _sharedData.requestServoStep = true;
  taskEXIT_CRITICAL(&_sharedData.spinlock);

  // Directly perform calculations and UDP sending on Core 1
  if (_com != nullptr) {
    // Send target details
    if (targetDetectedFinal) {
      _com->sendTarget(targetRangeFinal, currentAngle, targetStrengthFinal, velocity);
    }
    
    // Send frame
    _com->sendFrame(_waveFrameId++, localBuffer, Constant::ADC_SAMPLES, currentAngle);
  }

  // Reset accumulation for the next cycle
  _accumulatedCount = 0;
  _peakIdxStored = -1;
  taskENTER_CRITICAL(&_sharedData.spinlock);
  _sharedData.pulseIndex = 0;
  _sharedData.processingDone = true; // Signal transmitter that we are done with everything
  taskEXIT_CRITICAL(&_sharedData.spinlock);
}

float ReceiverApp::calculateVelocity(int peakIndex) {
  return 0.0f;
}
