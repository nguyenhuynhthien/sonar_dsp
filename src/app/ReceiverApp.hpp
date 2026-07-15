#ifndef RECEIVER_APP_HPP
#define RECEIVER_APP_HPP

#include <SharedSonarData.h>

class ComManager;

class ReceiverApp {
public:
    ReceiverApp(SharedSonarData& sharedData, uint16_t* adcBuffer, int receiverIndex = 0);
    void begin();
    void run();
    int32_t calculateVelocity(int peakIndex);
    void setComManager(ComManager& com) { _com = &com; }

private:
    SharedSonarData& _sharedData;
    uint16_t* _adcBuffer;
    int _receiverIndex;
    
    // DSP buffers (dynamically allocated if needed)
    int16_t* _demodI = nullptr;
    int16_t* _demodQ = nullptr;
    int16_t* _filteredI = nullptr;
    int16_t* _filteredQ = nullptr;
    int16_t* _tempConv1 = nullptr;
    int16_t* _tempConv2 = nullptr;

    // Matched filter coefficients (up to 104 samples for Barker 13)
    int16_t _hI[Constant::BARKER13_PULSE_LEN];
    int16_t _hQ[Constant::BARKER13_PULSE_LEN];
    int _filterLen = 32;

    // Slow-time history for Pulse-Doppler (16 pings)
    int16_t* _slowTimeI[Constant::SLOW_TIME_LEN] = {nullptr};
    int16_t* _slowTimeQ[Constant::SLOW_TIME_LEN] = {nullptr};
    int _pingCounter;

    // Pulse accumulation variables
    int16_t _fftReal[Constant::DOPPLER_FFT_LEN];
    int16_t _fftImag[Constant::DOPPLER_FFT_LEN];
    int _peakIdxStored = -1;
    int32_t* _accumulatedRaw = nullptr;
    int _accumulatedCount = 0;

    // Internal DSP routines
    void initDSPCoefficients();
    void performIQDemodulation(const int16_t* rawSamples);
    void performMatchedFiltering();
    void performPulseDopplerFFT();


    ComManager* _com = nullptr;
    uint16_t _waveFrameId = 0;
};

#endif // RECEIVER_APP_HPP
