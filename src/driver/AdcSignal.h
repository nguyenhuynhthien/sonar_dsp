#ifndef ADC_SIGNAL_H
#define ADC_SIGNAL_H

#include <Arduino.h>

class AdcSignal {
public:
    static void init();
    static uint16_t readRaw();
    static void startConversion();
    static uint16_t readResult();
};

#endif // ADC_SIGNAL_H
