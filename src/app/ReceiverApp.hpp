#ifndef RECEIVER_APP_HPP
#define RECEIVER_APP_HPP

#include <SharedSonarData.h>

class ComManager;

class ReceiverApp {
public:
    ReceiverApp(SharedSonarData& sharedData, uint16_t* adcBuffer, int receiverIndex = 0);
    void begin();
    void run();
    void setComManager(ComManager& com) { _com = &com; }

private:
    SharedSonarData& _sharedData;
    uint16_t* _adcBuffer;
    int _receiverIndex;
    
    // DSP buffers pointing to shared memory
    int16_t* _demodI = nullptr;
    int16_t* _demodQ = nullptr;

    // Matched filter coefficients (up to 104 samples for Barker 13)
    int16_t _hI[Constant::BARKER13_PULSE_LEN];
    int16_t _hQ[Constant::BARKER13_PULSE_LEN];
    int _filterLen = 32;

    // Internal DSP routines
    void initDSPCoefficients();
    void performIQDemodulation(const int16_t* rawSamples);
    void performMatchedFiltering(int pulseIdx);

    ComManager* _com = nullptr;
    uint16_t _waveFrameId = 0;
};

#endif // RECEIVER_APP_HPP
