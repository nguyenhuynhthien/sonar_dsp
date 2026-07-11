#ifndef ADC_SERVICE_H
#define ADC_SERVICE_H

#include <Arduino.h>

class AdcService {
public:
    static void init();
    static void sampleBuffer(uint16_t* buffer, size_t size, const uint8_t* txBuffer, size_t txPulseLen, volatile bool& adcReady);
};

#endif // ADC_SERVICE_H
