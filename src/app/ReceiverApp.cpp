#include "ReceiverApp.hpp"
#include "../service/ComManager.h"
#include "../service/DacService.h"
#include "../service/SyncSignalService.h"
#include <math.h>
#include <esp_dsp.h>

// Custom Q15 convolution implementation
static esp_err_t dsps_conv_q15(const int16_t *Signal, const int siglen, const int16_t *Kernel, const int kernlen, int16_t *convout) {
    int out_len = siglen + kernlen - 1;
    for (int n = 0; n < out_len; ++n) {
        int32_t sum = 0;
        int k_start = (n >= siglen) ? (n - siglen + 1) : 0;
        int k_end = (n < kernlen) ? n : (kernlen - 1);
        for (int k = k_start; k <= k_end; ++k) {
            sum += ((int32_t)Signal[n - k] * Kernel[k] + 16384) >> 15;
        }
        convout[n] = (int16_t)constrain(sum, -32768, 32767);
    }
    return ESP_OK;
}

ReceiverApp::ReceiverApp(SharedSonarData &sharedData, uint16_t* adcBuffer, int receiverIndex)
    : _sharedData(sharedData), _adcBuffer(adcBuffer), _receiverIndex(receiverIndex), _pingCounter(0) {}

void ReceiverApp::begin() {
  size_t convSize = Constant::ADC_SAMPLES + Constant::BARKER13_PULSE_LEN;
  _demodI = new (std::nothrow) int16_t[Constant::ADC_SAMPLES];
  _demodQ = new (std::nothrow) int16_t[Constant::ADC_SAMPLES];
  _filteredI = new (std::nothrow) int16_t[convSize];
  _filteredQ = new (std::nothrow) int16_t[convSize];
  _tempConv1 = new (std::nothrow) int16_t[convSize];
  _tempConv2 = new (std::nothrow) int16_t[convSize];
  _accumulatedRaw = new (std::nothrow) int32_t[Constant::ADC_SAMPLES];

  bool allocSuccess = _demodI && _demodQ && _filteredI && _filteredQ && 
                      _tempConv1 && _tempConv2 && _accumulatedRaw;

  if (!allocSuccess) {
    Serial.println("Error: Failed to allocate Q15 DSP/accumulation buffers on heap!");
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
    int16_t x = (int16_t)localTxBuffer[i] - Constant::DAC_DC_BIAS; // Keep original 8-bit scale [-128, 127]

    int16_t refCos = 0;
    int16_t refSin = 0;
    switch (i & 3) {
    case 0:
      refCos = Constant::Q15_MAX;
      refSin = 0;
      break;
    case 1:
      refCos = 0;
      refSin = Constant::Q15_MAX;
      break;
    case 2:
      refCos = Constant::Q15_MIN;
      refSin = 0;
      break;
    case 3:
      refCos = 0;
      refSin = Constant::Q15_MIN;
      break;
    }

    // matched filter: h(t) = s*(T - t)
    int filterIdx = pulseLen - 1 - i;
    _hI[filterIdx] = (int16_t)(((int32_t)x * refCos) >> 15);
    _hQ[filterIdx] = - (int16_t)(((int32_t)x * refSin) >> 15); // conjugate
  }

}

void ReceiverApp::performIQDemodulation(const int16_t *rawSamples) {
  if (!_demodI || !_demodQ)
    return;

  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    int16_t x = rawSamples[n];
    switch (n & 3) {
    case 0:
      _demodI[n] = x;
      _demodQ[n] = 0;
      break;
    case 1:
      _demodI[n] = 0;
      _demodQ[n] = -x;
      break;
    case 2:
      _demodI[n] = -x;
      _demodQ[n] = 0;
      break;
    case 3:
      _demodI[n] = 0;
      _demodQ[n] = x;
      break;
    }
  }

  // Apply 4-sample Moving Average FIR filter to smooth demodulated signals and remove carrier ripple
  static int16_t tempDemodI[Constant::ADC_SAMPLES];
  static int16_t tempDemodQ[Constant::ADC_SAMPLES];
  memcpy(tempDemodI, _demodI, sizeof(tempDemodI));
  memcpy(tempDemodQ, _demodQ, sizeof(tempDemodQ));

  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    int32_t sumDemodI = 0;
    int32_t sumDemodQ = 0;
    for (int d = 0; d < 4; ++d) {
      if (n - d >= 0) {
        sumDemodI += tempDemodI[n - d];
        sumDemodQ += tempDemodQ[n - d];
      }
    }
    _demodI[n] = (int16_t)(sumDemodI >> 2);
    _demodQ[n] = (int16_t)(sumDemodQ >> 2);
  }
}

void ReceiverApp::performMatchedFiltering() {
  if (!_demodI || !_demodQ || !_filteredI || !_filteredQ)
    return;

  // Real-time matched filtering in a single optimized pass.
  // Since coefficients represent carrier sines/cosines at 40kHz/160kHz,
  // either coefI or coefQ is zero at any index. We skip zero multiplications to save 4x operations.
  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    int32_t sumI = 0;
    int32_t sumQ = 0;

    for (int k = 0; k < _filterLen; ++k) {
      if (n >= k) {
        int16_t sigI = _demodI[n - k];
        int16_t sigQ = _demodQ[n - k];
        int16_t coefI = _hI[k];
        int16_t coefQ = _hQ[k];

        if (coefI != 0) {
          sumI += (int32_t)sigI * coefI;
          sumQ += (int32_t)sigQ * coefI;
        }
        if (coefQ != 0) {
          sumI -= (int32_t)sigQ * coefQ;
          sumQ += (int32_t)sigI * coefQ;
        }
      }
    }

    _filteredI[n] = (int16_t)constrain((sumI + Constant::MATCHED_FILTER_ROUND_OFFSET) >> Constant::MATCHED_FILTER_SHIFT, Constant::Q15_MIN, Constant::Q15_MAX);
    _filteredQ[n] = (int16_t)constrain((sumQ + Constant::MATCHED_FILTER_ROUND_OFFSET) >> Constant::MATCHED_FILTER_SHIFT, Constant::Q15_MIN, Constant::Q15_MAX);
  }
}

// Fast integer square root helper
static uint32_t isqrt32(uint32_t n) {
    uint32_t res = 0;
    uint32_t bit = 1u << 30; // The second-to-top bit is set
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

void ReceiverApp::run() {
  // Dynamically update matched filter coefficients if pulse type changed
  initDSPCoefficients();

  // Capture the raw samples under lock
  static uint16_t localRawSamples[Constant::ADC_SAMPLES];
  taskENTER_CRITICAL(&_sharedData.spinlock);
  memcpy(localRawSamples, (const void *)_adcBuffer,
         sizeof(localRawSamples));
  taskEXIT_CRITICAL(&_sharedData.spinlock);

  // 1. Digital DC Filter: compute the exact mean of the 2048 samples
  int32_t sum = 0;
  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    sum += localRawSamples[n];
  }
  int16_t mean = sum / (int)Constant::ADC_SAMPLES;

  // 2. DC-filter the pulse to a local temp array and convert to Q15 by shifting left by 4
  static int16_t tempRaw[Constant::ADC_SAMPLES];
  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    tempRaw[n] = (int16_t)(localRawSamples[n] - mean) << 4;
  }

  // 3. Process IQ Demodulation and Matched Filtering for the current pulse
  performIQDemodulation(tempRaw);
  performMatchedFiltering();

  // Initialize accumulated raw values array if first pulse
  if (_accumulatedCount == 0) {
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
      _accumulatedRaw[n] = ((int32_t)_filteredI[n] * _filteredI[n] + (int32_t)_filteredQ[n] * _filteredQ[n]) >> 4;
    }
  } else {
    // Accumulate incoherent energy for smoother peak detection
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
      _accumulatedRaw[n] += ((int32_t)_filteredI[n] * _filteredI[n] + (int32_t)_filteredQ[n] * _filteredQ[n]) >> 4;
    }
  }

  // Store the complex samples of the ping history for ALL indices (or just around the peak?)
  // To keep it simple and preserve current architecture, we'll find peak later.
  // Actually, we must store the complex values at ALL indices for all pulses to find the peak correctly later,
  // but we don't have enough memory for [16][2048].
  // Strategy: Picking peak at pulse #4 (middle of accumulation) or first pulse. 
  // Let's improve the first-pulse peak detection to be at least a bit more robust or wait for more pulses.
  
  // Improvement: Always use a consistent peak index for the entire batch of 8 pulses.
  // We only detect the peak update once at the start of accumulation to maintain phase coherence.
  if (_accumulatedCount == 4 || (_accumulatedCount == 0 && _peakIdxStored == -1)) {
    int32_t maxAcc = 0;
    int bestIdx = -1;
    for (int n = Constant::TX_LEAKAGE_BLANK_SAMPLES; n < (int)Constant::ADC_SAMPLES; ++n) {
        if (_accumulatedRaw[n] > maxAcc) {
            maxAcc = _accumulatedRaw[n];
            bestIdx = n;
        }
    }
    _peakIdxStored = bestIdx;

    if (_peakIdxStored != -1) {
      int32_t dspGain = (_filterLen == (int)Constant::BARKER13_PULSE_LEN) ? 
                        ((int32_t)(Constant::BARKER13_PULSE_LEN / 2) * Constant::DAC_DC_BIAS) : 
                        ((int32_t)(Constant::FILTER_COEFFS_LEN / 2) * Constant::DAC_DC_BIAS);
      int32_t baseThreshold = (_filterLen == (int)Constant::BARKER13_PULSE_LEN) ? Constant::BASE_BARKER13_THRESHOLD : Constant::BASE_SINGLE_PULSE_THRESHOLD;
      int32_t localThreshold = baseThreshold << (14 - Constant::MATCHED_FILTER_SHIFT);
      int64_t threshVal = (int64_t)localThreshold * dspGain;
      
      int numPings = _accumulatedCount + 1;
      int64_t estimatedMaxValSq = ((int64_t)maxAcc << 4) / numPings;
      if (estimatedMaxValSq * Constant::TARGET_THRESHOLD_SCALE_SQ < threshVal * threshVal) {
        _peakIdxStored = -1; // Under threshold, reject target
      }
    }
  }

  // Extract the complex value of this ping at the CURRENTLY detected peak
  if (_peakIdxStored != -1) {
    _fftReal[_accumulatedCount] = _filteredI[_peakIdxStored];
    _fftImag[_accumulatedCount] = _filteredQ[_peakIdxStored];
  } else {
    _fftReal[_accumulatedCount] = 0;
    _fftImag[_accumulatedCount] = 0;
  }

  _accumulatedCount++;

  if (_accumulatedCount < 8) {
    // Not enough pulses accumulated yet (using 8 pulses to save RAM/Time)
    taskENTER_CRITICAL(&_sharedData.spinlock);
    _sharedData.pulseIndex = _accumulatedCount;
    _sharedData.processingDone |= (1 << _receiverIndex);
    taskEXIT_CRITICAL(&_sharedData.spinlock);
    return;
  }

  // We have integrated 8 pulses. 
  // Step 2: Quét ngang chạy FFT 16 điểm (8 data points + 8 zero padding)
  int32_t velocity_bin = 0;
  int32_t cleanRawMag = 0;
  
  // Final check on target detection using integrated energy threshold
  bool targetDetected = (_peakIdxStored != -1);

  if (targetDetected) {
    // Run 16-point DFT with Zero Padding (8 data + 8 zeros)
    int32_t realOut[Constant::DOPPLER_FFT_LEN] = {0};
    int32_t imagOut[Constant::DOPPLER_FFT_LEN] = {0};
    
    for (int k = 0; k < (int)Constant::DOPPLER_FFT_LEN; ++k) { 
      float sumReal = 0;
      float sumImag = 0;
      // Only iterate up to 8 for the data pings (N=8 data points)
      for (int n = 0; n < 8; ++n) {
        float angle = -2.0f * M_PI * k * n / (float)Constant::DOPPLER_FFT_LEN;
        float cosVal = cosf(angle);
        float sinVal = sinf(angle);

        sumReal += (float)_fftReal[n] * cosVal - (float)_fftImag[n] * sinVal;
        sumImag += (float)_fftReal[n] * sinVal + (float)_fftImag[n] * cosVal;
      }
      realOut[k] = (int32_t)sumReal;
      imagOut[k] = (int32_t)sumImag;
    }

    // Find peak frequency bin
    velocity_bin = 0;
    int64_t maxFftMag = -1;
    for (int k = 0; k < (int)Constant::DOPPLER_FFT_LEN; ++k) {
      int64_t fftMag = (int64_t)realOut[k] * realOut[k] + (int64_t)imagOut[k] * imagOut[k];
      if (fftMag > maxFftMag) {
        maxFftMag = fftMag;
        velocity_bin = k;
      }
    }

    // Convert circular bin index to signed Doppler index (-N/2 to N/2-1)
    if (velocity_bin >= (int)Constant::DOPPLER_FFT_LEN / 2) {
        velocity_bin -= (int)Constant::DOPPLER_FFT_LEN;
    }

    cleanRawMag = (int32_t)sqrtf((float)maxFftMag);
  }

  // Copy streaming values to localBuffer
  uint8_t mode = 0;
  taskENTER_CRITICAL(&_sharedData.spinlock);
  mode = _sharedData.streamMode;
  taskEXIT_CRITICAL(&_sharedData.spinlock);

  static int16_t localBuffer[Constant::ADC_SAMPLES];
  if (mode == 0) { // STREAM_RAW
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
      localBuffer[n] = tempRaw[n];
    }
  } else if (mode == 1) { // STREAM_DEMOD
    performIQDemodulation(tempRaw);
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
      int32_t sqVal = (int32_t)_demodI[n] * _demodI[n] + (int32_t)_demodQ[n] * _demodQ[n];
      localBuffer[n] = (int16_t)constrain(isqrt32(sqVal), 0, Constant::Q15_MAX);
    }
  } else if (mode == 2) { // STREAM_COMPRESSED
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
      localBuffer[n] = (int16_t)constrain(isqrt32(2 * _accumulatedRaw[n]), 0, Constant::Q15_MAX);
    }
  }

  // Copy localBuffer to old adcBuffer for general tracking
  memcpy((void*)_adcBuffer, localBuffer, sizeof(localBuffer));
  
  uint16_t currentAngle = 0;
  taskENTER_CRITICAL(&_sharedData.spinlock);
  currentAngle = _sharedData.servoAngle;
  taskEXIT_CRITICAL(&_sharedData.spinlock);

  // Directly perform calculations and UDP sending on Core 1
  if (_com != nullptr) {
    // Send target details
    if (targetDetected && _peakIdxStored != -1) {
      _com->sendTarget(_peakIdxStored, currentAngle, cleanRawMag, velocity_bin, _receiverIndex);
    }
    
    // Send frame
    _com->sendFrame(_waveFrameId++, localBuffer, Constant::ADC_SAMPLES, currentAngle, _receiverIndex);
  }

  // Reset accumulation for the next cycle
  _accumulatedCount = 0;
  _peakIdxStored = -1;

  taskENTER_CRITICAL(&_sharedData.spinlock);
  _sharedData.processingDone |= (1 << _receiverIndex);
  bool bothDone = (_sharedData.processingDone == 3);
  if (bothDone) {
    _sharedData.pulseIndex = 0;
    _sharedData.accumulatedDataReady = true;
    _sharedData.requestServoStep = true;
  }
  taskEXIT_CRITICAL(&_sharedData.spinlock);
}

int32_t ReceiverApp::calculateVelocity(int peakIndex) {
  return 0;
}
