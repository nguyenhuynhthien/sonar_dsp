#ifndef RECEIVER_APP_HPP
#define RECEIVER_APP_HPP

#include <Arduino.h>
#include <Constant.hpp>
#include "TransmitterApp.h"



class ReceiverApp {
public:
    ReceiverApp(SharedSonarData& sharedData);
    void begin();
    void run();

private:
    SharedSonarData& _sharedData;
    
    // DSP buffers (dynamically allocated if needed)
    float* _demodI = nullptr;
    float* _demodQ = nullptr;
    float* _filteredI = nullptr;
    float* _filteredQ = nullptr;

    // Matched filter coefficients (32 samples)
    float _hI[Constant::FILTER_COEFFS_LEN];
    float _hQ[Constant::FILTER_COEFFS_LEN];

    // Slow-time history for Pulse-Doppler (16 pings)
    float* _slowTimeI[Constant::SLOW_TIME_LEN] = {nullptr};
    float* _slowTimeQ[Constant::SLOW_TIME_LEN] = {nullptr};
    int _pingCounter;

    // Internal DSP routines
    void initDSPCoefficients();
    void performIQDemodulation(const uint16_t* rawSamples);
    void performMatchedFiltering();
    void performPulseDopplerFFT();
    void fftRadix2(float* real, float* imag, int n);
};

#endif // RECEIVER_APP_HPP
