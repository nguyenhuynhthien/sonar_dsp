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

// Static buffers for receiver apps
static int16_t s_tempRaw[Constant::ADC_SAMPLES];
static int16_t s_localBuffer[2][Constant::ADC_SAMPLES];

static inline int16_t* getChannelTempOutI(int rxIdx, SharedSonarData& shared) {
    return (rxIdx == 0) ? (int16_t*)&shared.dsp_scratchpad[0][0] : (int16_t*)&shared.dsp_scratchpad[1024][0];
}

static inline int16_t* getChannelTempOutQ(int rxIdx, SharedSonarData& shared) {
    return (rxIdx == 0) ? (int16_t*)&shared.dsp_scratchpad[512][0] : (int16_t*)&shared.dsp_scratchpad[1536][0];
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

  // 4-sample moving average filter using rolling sums without dynamic memory
  int32_t sumI = 0, sumQ = 0;
  for (int n = 0; n < (int)Constant::ADC_SAMPLES + 3; ++n) {
    if (n < (int)Constant::ADC_SAMPLES) {
      sumI += _demodI[n];
      sumQ += _demodQ[n];
    }
    if (n >= 4) {
      sumI -= _demodI[n - 4];
      sumQ -= _demodQ[n - 4];
    }
    if (n >= 3 && (n - 3) < (int)Constant::ADC_SAMPLES) {
      _demodI[n - 3] = (int16_t)(sumI >> 2);
      _demodQ[n - 3] = (int16_t)(sumQ >> 2);
    }
  }
}

void ReceiverApp::performMatchedFiltering(int pulseIdx) {
  if (!_demodI || !_demodQ)
    return;

  int16_t* outI = (_receiverIndex == 0) ? _sharedData.channelL_I[pulseIdx] : _sharedData.channelR_I[pulseIdx];
  int16_t* outQ = (_receiverIndex == 0) ? _sharedData.channelL_Q[pulseIdx] : _sharedData.channelR_Q[pulseIdx];

  int rxIdx = (_receiverIndex >= 0 && _receiverIndex < 2) ? _receiverIndex : 0;
  int16_t* channelTempOutI = getChannelTempOutI(rxIdx, _sharedData);
  int16_t* channelTempOutQ = getChannelTempOutQ(rxIdx, _sharedData);

  if (pulseIdx == 0) {
    // 1. Compute full matched filter for Pulse 0 into dsp_scratchpad for this channel
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

      channelTempOutI[n] = (int16_t)constrain((sumI + Constant::MATCHED_FILTER_ROUND_OFFSET) >> Constant::MATCHED_FILTER_SHIFT, Constant::Q15_MIN, Constant::Q15_MAX);
      channelTempOutQ[n] = (int16_t)constrain((sumQ + Constant::MATCHED_FILTER_ROUND_OFFSET) >> Constant::MATCHED_FILTER_SHIFT, Constant::Q15_MIN, Constant::Q15_MAX);
    }

    // 2. Find peak index for Pulse 0 on this channel
    int bestIdx = -1;
    int32_t maxEnergy = 0;
    for (int n = Constant::TX_LEAKAGE_BLANK_SAMPLES; n < (int)Constant::ADC_SAMPLES; ++n) {
      int32_t energy = ((int32_t)channelTempOutI[n] * channelTempOutI[n] + 
                        (int32_t)channelTempOutQ[n] * channelTempOutQ[n]) >> 4;
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
        outI[k] = channelTempOutI[n];
        outQ[k] = channelTempOutQ[n];
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
  initDSPCoefficients();

  int idx = (_receiverIndex >= 0 && _receiverIndex < 2) ? _receiverIndex : 0;

  // 1. Digital DC Filter: compute mean directly from _adcBuffer under lock
  int32_t sum = 0;
  taskENTER_CRITICAL(&_sharedData.spinlock);
  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    sum += _adcBuffer[n];
  }
  int16_t mean = sum / (int)Constant::ADC_SAMPLES;

  // 2. DC-filter into s_tempRaw
  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    int32_t val = ((int32_t)_adcBuffer[n] - mean) << 4;
    s_tempRaw[n] = (int16_t)constrain(val, Constant::Q15_MIN, Constant::Q15_MAX);
  }
  taskEXIT_CRITICAL(&_sharedData.spinlock);

  // Get current pulse index
  int pulseIdx = _sharedData.pulseIndex;

  // 3. Process IQ Demodulation and Matched Filtering for current pulse
  performIQDemodulation(s_tempRaw);
  performMatchedFiltering(pulseIdx);

  // Copy streaming values ONLY for Pulse 0
  if (pulseIdx == 0) {
    uint8_t mode = 0;
    taskENTER_CRITICAL(&_sharedData.spinlock);
    mode = _sharedData.streamMode;
    taskEXIT_CRITICAL(&_sharedData.spinlock);

    if (mode == 0) { // STREAM_RAW
      for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
        s_localBuffer[idx][n] = s_tempRaw[n];
      }
    } else if (mode == 1) { // STREAM_DEMOD
      performIQDemodulation(s_tempRaw);
      for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
        int32_t sqVal = (int32_t)_demodI[n] * _demodI[n] + (int32_t)_demodQ[n] * _demodQ[n];
        s_localBuffer[idx][n] = (int16_t)constrain(isqrt32(sqVal), 0, Constant::Q15_MAX);
      }
    } else if (mode == 2) { // STREAM_COMPRESSED
      int16_t* channelTempOutI = getChannelTempOutI(idx, _sharedData);
      int16_t* channelTempOutQ = getChannelTempOutQ(idx, _sharedData);
      for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
        int32_t sqVal = (int32_t)channelTempOutI[n] * channelTempOutI[n] + 
                          (int32_t)channelTempOutQ[n] * channelTempOutQ[n];
        s_localBuffer[idx][n] = (int16_t)constrain(isqrt32(sqVal >> 4), 0, Constant::Q15_MAX);
      }
    }

    if (_com != nullptr) {
      _com->sendFrameAsync(_waveFrameId++, s_localBuffer[idx], Constant::ADC_SAMPLES, _receiverIndex + 1);
    }
  }

  // Signal completion
  taskENTER_CRITICAL(&_sharedData.spinlock);
  _sharedData.processingDone |= (1 << _receiverIndex);
  taskEXIT_CRITICAL(&_sharedData.spinlock);
}



