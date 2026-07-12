#ifndef ADC_SERVICE_H
#define ADC_SERVICE_H

#include <Arduino.h>

#include <driver/dac.h>
#include "DacService.h"

class AdcService {
public:
    static void init();
    static void sampleBuffer(uint16_t* buffer, size_t size);
};

#endif // ADC_SERVICE_H
