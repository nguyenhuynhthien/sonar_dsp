#ifndef RECEIVER_APP_HPP
#define RECEIVER_APP_HPP

#include <SharedSonarData.h>

class ComManager;

class ReceiverApp {
public:
    ReceiverApp(SharedSonarData& sharedData);
    void begin();
    void run();
    float calculateVelocity(int peakIndex);
    void setComManager(ComManager& com) { _com = &com; }

private:
    SharedSonarData& _sharedData;
    
    // DSP buffers (dynamically allocated if needed)
    float* _demodI = nullptr;
    float* _demodQ = nullptr;
    float* _filteredI = nullptr;
    float* _filteredQ = nullptr;

    // Matched filter coefficients (up to 104 samples for Barker 13)
    float _hI[Constant::BARKER13_PULSE_LEN];
    float _hQ[Constant::BARKER13_PULSE_LEN];
    int _filterLen = 32;

    // Slow-time history for Pulse-Doppler (16 pings)
    float* _slowTimeI[Constant::SLOW_TIME_LEN] = {nullptr};
    float* _slowTimeQ[Constant::SLOW_TIME_LEN] = {nullptr};
    int _pingCounter;

    // Pulse accumulation variables
    float _fftReal[8];
    float _fftImag[8];
    int _peakIdxStored = -1;
    float* _accumulatedRaw = nullptr;
    int _accumulatedCount = 0;

    // Internal DSP routines
    void initDSPCoefficients();
    void performIQDemodulation(const float* rawSamples);
    void performMatchedFiltering();
    void performPulseDopplerFFT();
    void fftRadix2(float* real, float* imag, int n);

    ComManager* _com = nullptr;
    uint16_t _waveFrameId = 0;
};

#endif // RECEIVER_APP_HPP
