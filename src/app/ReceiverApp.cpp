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
    : _sharedData(sharedData), _adcBuffer(adcBuffer), _receiverIndex(receiverIndex) {}

void ReceiverApp::begin() {
  _demodI = _sharedData.sharedDemodI;
  _demodQ = _sharedData.sharedDemodQ;

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

int16_t s_tempOutI[2][Constant::ADC_SAMPLES];
int16_t s_tempOutQ[2][Constant::ADC_SAMPLES];

void ReceiverApp::performMatchedFiltering(int pulseIdx) {
  if (!_demodI || !_demodQ)
    return;

  int16_t* outI = (_receiverIndex == 0) ? _sharedData.channelL_I[pulseIdx] : _sharedData.channelR_I[pulseIdx];
  int16_t* outQ = (_receiverIndex == 0) ? _sharedData.channelL_Q[pulseIdx] : _sharedData.channelR_Q[pulseIdx];

  if (pulseIdx == 0) {
    // 1. Compute full matched filter for Pulse 0 into file-scope static buffer
    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
      int32_t sumI = 0;
      int32_t sumQ = 0;

      int maxK = (n < _filterLen) ? (n + 1) : _filterLen;
      for (int k = 0; k < maxK; ++k) {
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

      s_tempOutI[_receiverIndex][n] = (int16_t)constrain((sumI + Constant::MATCHED_FILTER_ROUND_OFFSET) >> Constant::MATCHED_FILTER_SHIFT, Constant::Q15_MIN, Constant::Q15_MAX);
      s_tempOutQ[_receiverIndex][n] = (int16_t)constrain((sumQ + Constant::MATCHED_FILTER_ROUND_OFFSET) >> Constant::MATCHED_FILTER_SHIFT, Constant::Q15_MIN, Constant::Q15_MAX);
    }

    // 2. Find peak index for Pulse 0 on this channel
    int bestIdx = -1;
    int32_t maxEnergy = 0;
    for (int n = Constant::TX_LEAKAGE_BLANK_SAMPLES; n < (int)Constant::ADC_SAMPLES; ++n) {
      int32_t energy = ((int32_t)s_tempOutI[_receiverIndex][n] * s_tempOutI[_receiverIndex][n] + 
                        (int32_t)s_tempOutQ[_receiverIndex][n] * s_tempOutQ[_receiverIndex][n]) >> 4;
      if (energy > maxEnergy) {
        maxEnergy = energy;
        bestIdx = n;
      }
    }

    // 3. Update the shared window center index under lock
    if (bestIdx != -1 && _receiverIndex == 0) {
      taskENTER_CRITICAL(&_sharedData.spinlock);
      _sharedData.sharedWindowCenterIdx = bestIdx;
      taskEXIT_CRITICAL(&_sharedData.spinlock);
    }

    // 4. Extract window of 15 samples centered around sharedWindowCenterIdx
    int center = 0;
    taskENTER_CRITICAL(&_sharedData.spinlock);
    center = _sharedData.sharedWindowCenterIdx;
    taskEXIT_CRITICAL(&_sharedData.spinlock);

    for (int k = 0; k < Constant::FFT_WINDOW_SIZE; ++k) {
      int n = center - (Constant::FFT_WINDOW_SIZE / 2) + k;
      if (n >= 0 && n < (int)Constant::ADC_SAMPLES) {
        outI[k] = s_tempOutI[_receiverIndex][n];
        outQ[k] = s_tempOutQ[_receiverIndex][n];
      } else {
        outI[k] = 0;
        outQ[k] = 0;
      }
    }
  } else {
    // Pulse 1 to 7: Compute matched filter only at the 15 windowed samples
    int center = 0;
    taskENTER_CRITICAL(&_sharedData.spinlock);
    center = _sharedData.sharedWindowCenterIdx;
    taskEXIT_CRITICAL(&_sharedData.spinlock);

    for (int k = 0; k < Constant::FFT_WINDOW_SIZE; ++k) {
      int n = center - (Constant::FFT_WINDOW_SIZE / 2) + k;
      if (n >= 0 && n < (int)Constant::ADC_SAMPLES) {
        int32_t sumI = 0;
        int32_t sumQ = 0;

        int maxK = (n < _filterLen) ? (n + 1) : _filterLen;
        for (int k_filt = 0; k_filt < maxK; ++k_filt) {
          int16_t sigI = _demodI[n - k_filt];
          int16_t sigQ = _demodQ[n - k_filt];
          int16_t coefI = _hI[k_filt];
          int16_t coefQ = _hQ[k_filt];

          if (coefI != 0) {
            sumI += (int32_t)sigI * coefI;
            sumQ += (int32_t)sigQ * coefI;
          }
          if (coefQ != 0) {
            sumI -= (int32_t)sigQ * coefQ;
            sumQ += (int32_t)sigI * coefQ;
          }
        }

        outI[k] = (int16_t)constrain((sumI + Constant::MATCHED_FILTER_ROUND_OFFSET) >> Constant::MATCHED_FILTER_SHIFT, Constant::Q15_MIN, Constant::Q15_MAX);
        outQ[k] = (int16_t)constrain((sumQ + Constant::MATCHED_FILTER_ROUND_OFFSET) >> Constant::MATCHED_FILTER_SHIFT, Constant::Q15_MIN, Constant::Q15_MAX);
      } else {
        outI[k] = 0;
        outQ[k] = 0;
      }
    }
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

  // Get current pulse index
  int pulseIdx = _sharedData.pulseIndex;

  // 3. Process IQ Demodulation and Matched Filtering for the current pulse
  performIQDemodulation(tempRaw);
  performMatchedFiltering(pulseIdx);

  // Copy streaming values to localBuffer ONLY for Pulse 0 to save bandwidth
  if (pulseIdx == 0) {
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
        int32_t sqVal = (int32_t)s_tempOutI[_receiverIndex][n] * s_tempOutI[_receiverIndex][n] + 
                          (int32_t)s_tempOutQ[_receiverIndex][n] * s_tempOutQ[_receiverIndex][n];
        localBuffer[n] = (int16_t)constrain(isqrt32(sqVal >> 4), 0, Constant::Q15_MAX);
      }
    }

    // Copy localBuffer to old adcBuffer for general tracking
    memcpy((void*)_adcBuffer, localBuffer, sizeof(localBuffer));
    
    uint16_t currentAngle = 0;
    bool isCCW = true;
    taskENTER_CRITICAL(&_sharedData.spinlock);
    currentAngle = _sharedData.servoAngle;
    isCCW = _sharedData.sweepDirectionCCW;
    taskEXIT_CRITICAL(&_sharedData.spinlock);

    if (_com != nullptr) {
      uint16_t angleToSend = currentAngle;
      if (!isCCW) angleToSend |= 0x8000;
      if (_waveFrameId % 3 == 0) {
        _com->sendFrameAsync(_waveFrameId, localBuffer, Constant::ADC_SAMPLES, angleToSend, _receiverIndex + 1); // receiverId = 1 or 2
      }
      _waveFrameId++;
    }
  }

  // Signal that this receiver is done processing the current pulse
  taskENTER_CRITICAL(&_sharedData.spinlock);
  _sharedData.processingDone |= (1 << _receiverIndex);
  taskEXIT_CRITICAL(&_sharedData.spinlock);
}
