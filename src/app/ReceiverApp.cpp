#include "ReceiverApp.hpp"
#include "../service/ComManager.h"
#include "../service/DacService.h"
#include "../service/SyncSignalService.h"
#include <math.h>
#include <esp_dsp.h>
#include "../service/XtensaDspSimdHelper.h"



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
  _numTapsI = 0;
  _numTapsQ = 0;

  // Pre-calculate sparse non-zero taps for time-reversed conjugate matched filter h(t) = s*(T - t)
  for (int i = 0; i < pulseLen; ++i) {
    int16_t x = (int16_t)localTxBuffer[i] - Constant::DAC_DC_BIAS;

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

    int filterIdx = pulseLen - 1 - i;
    int16_t hI = (int16_t)(((int32_t)x * refCos) >> 15);
    int16_t hQ = - (int16_t)(((int32_t)x * refSin) >> 15);

    if (hI != 0) {
      _tapsI[_numTapsI++] = { (uint8_t)filterIdx, hI };
    }
    if (hQ != 0) {
      _tapsQ[_numTapsQ++] = { (uint8_t)filterIdx, hQ };
    }
  }

  // Dynamically calculate Matched Filter shift power-of-two exponent based on active filter taps without magic numbers
  int maxTaps = (_numTapsI > _numTapsQ) ? _numTapsI : _numTapsQ;
  if (maxTaps <= 0) maxTaps = 1;

  _matchedFilterShift = 0;
  while ((1 << _matchedFilterShift) < maxTaps) {
    _matchedFilterShift++;
  }
  // Base Q15 scale offset for optimal dynamic range (Single Pulse ~3.2V, Barker 13 ~11.5V)
  constexpr int BASE_Q15_SHIFT = 7;
  _matchedFilterShift += BASE_Q15_SHIFT;
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

  ae_int16x2* pDemodI = (ae_int16x2*)_demodI;
  ae_int16x2* pDemodQ = (ae_int16x2*)_demodQ;
  const ae_int16x2* pRaw = (const ae_int16x2*)rawSamples; // Căn chỉnh bộ nhớ 4-byte

  constexpr int SAMPLES_PER_REGISTER = sizeof(ae_int16x2) / sizeof(int16_t);

  for (int n = 0; n < (int)Constant::ADC_SAMPLES; n += Constant::DEMOD_SAMPLE_PERIOD) {
    ae_int16x2 r01 = pRaw[n / SAMPLES_PER_REGISTER];     // Chứa [ x1 | x0 ]
    ae_int16x2 r23 = pRaw[n / SAMPLES_PER_REGISTER + 1]; // Chứa [ x3 | x2 ]
    
    int16_t x0 = AE_MOV2X16_0(r01);
    int16_t x1 = AE_MOV2X16_1(r01);
    int16_t x2 = AE_MOV2X16_0(r23);
    int16_t x3 = AE_MOV2X16_1(r23);
    
    pDemodI[n / SAMPLES_PER_REGISTER] = AE_MOVDA16(0, x0);
    pDemodI[n / SAMPLES_PER_REGISTER + 1] = AE_MOVDA16(0, -x2);
    
    pDemodQ[n / SAMPLES_PER_REGISTER] = AE_MOVDA16(-x1, 0);
    pDemodQ[n / SAMPLES_PER_REGISTER + 1] = AE_MOVDA16(x3, 0);
  }

  // Bộ lọc trung bình trượt 4 mẫu sử dụng con trỏ dịch liên tục
  int32_t sumI = 0, sumQ = 0;
  int16_t* pInI = _demodI;
  int16_t* pInQ = _demodQ;
  int16_t* pOutI = _demodI;
  int16_t* pOutQ = _demodQ;
  int16_t* pSubI = _demodI;
  int16_t* pSubQ = _demodQ;

  for (int n = 0; n < (int)Constant::ADC_SAMPLES + (Constant::DEMOD_AVG_LEN - 1); ++n) {
    if (n < (int)Constant::ADC_SAMPLES) {
      sumI += *pInI++;
      sumQ += *pInQ++;
    }
    if (n >= Constant::DEMOD_AVG_LEN) {
      sumI -= *pSubI++;
      sumQ -= *pSubQ++;
    }
    if (n >= (Constant::DEMOD_AVG_LEN - 1) && (n - (Constant::DEMOD_AVG_LEN - 1)) < (int)Constant::ADC_SAMPLES) {
      *pOutI++ = (int16_t)(sumI >> 2); // dịch 2 bit tương đương chia cho DEMOD_AVG_LEN = 4
      *pOutQ++ = (int16_t)(sumQ >> 2);
    }
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

void ReceiverApp::performMatchedFiltering(int pulseIdx) {
  if (!_demodI || !_demodQ)
    return;

  int rxIdx = (_receiverIndex >= 0 && _receiverIndex < 2) ? _receiverIndex : 0;
  int16_t* channelTempOutI = getChannelTempOutI(rxIdx, _sharedData);
  int16_t* channelTempOutQ = getChannelTempOutQ(rxIdx, _sharedData);

  int32_t roundOffset = 1 << (_matchedFilterShift - 1);

  // 1. Ultra-fast sparse tap matched filtering (only iterates over non-zero coefficients)
  for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
    int32_t sumI = 0;
    int32_t sumQ = 0;

    for (int t = 0; t < _numTapsI; ++t) {
      int k = _tapsI[t].k;
      if (k > n) continue;
      int16_t cI = _tapsI[t].val;
      sumI += (int32_t)_demodI[n - k] * cI;
      sumQ += (int32_t)_demodQ[n - k] * cI;
    }

    for (int t = 0; t < _numTapsQ; ++t) {
      int k = _tapsQ[t].k;
      if (k > n) continue;
      int16_t cQ = _tapsQ[t].val;
      sumI -= (int32_t)_demodQ[n - k] * cQ;
      sumQ += (int32_t)_demodI[n - k] * cQ;
    }

    channelTempOutI[n] = (int16_t)constrain((sumI + roundOffset) >> _matchedFilterShift, Constant::Q15_MIN, Constant::Q15_MAX);
    channelTempOutQ[n] = (int16_t)constrain((sumQ + roundOffset) >> _matchedFilterShift, Constant::Q15_MIN, Constant::Q15_MAX);
  }








  // 2. When Rx2 (rxIdx == 1) finishes processing current pulse, update matrixSum_I/Q and accumulate diffAccumulator
  if (rxIdx == 1 && pulseIdx >= 0 && pulseIdx < 8) {
    int16_t* ch0_I = getChannelTempOutI(0, _sharedData);
    int16_t* ch0_Q = getChannelTempOutQ(0, _sharedData);
    int16_t* ch1_I = getChannelTempOutI(1, _sharedData);
    int16_t* ch1_Q = getChannelTempOutQ(1, _sharedData);

    for (int n = 0; n < (int)Constant::ADC_SAMPLES; ++n) {
      ae_int16x2 ch0 = AE_MOVDA16(ch0_Q[n], ch0_I[n]);
      ae_int16x2 ch1 = AE_MOVDA16(ch1_Q[n], ch1_I[n]);
      ae_int16x2 sum = AE_ADD16(ch0, ch1);
      ae_int16x2 diff = AE_SUB16(ch0, ch1);

      int16_t sumI_val = AE_MOV2X16_0(sum);
      int16_t sumQ_val = AE_MOV2X16_1(sum);
      int16_t diffI_val = AE_MOV2X16_0(diff);
      int16_t diffQ_val = AE_MOV2X16_1(diff);

      _sharedData.matrixSum_I[pulseIdx][n] = sumI_val >> 1;
      _sharedData.matrixSum_Q[pulseIdx][n] = sumQ_val >> 1;

      int32_t diffI = diffI_val;
      int32_t diffQ = diffQ_val;
      int32_t diffSq = (diffI * diffI + diffQ * diffQ) >> 4;
      int16_t diffMag = (int16_t)constrain(isqrt32(diffSq), 0, Constant::Q15_MAX);

      if (pulseIdx == 0) {
        _sharedData.diffAccumulator[n] = diffMag;
      } else {
        int32_t acc = (int32_t)_sharedData.diffAccumulator[n] + diffMag;
        _sharedData.diffAccumulator[n] = (int16_t)constrain(acc, 0, Constant::Q15_MAX);
      }
    }
  }
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

  // Copy streaming values ONLY for Pulse 0 (or Pulse 7 for accumulated diff)
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




