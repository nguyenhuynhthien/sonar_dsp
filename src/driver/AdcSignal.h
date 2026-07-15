#ifndef ADC_SIGNAL_H
#define ADC_SIGNAL_H

#include <Arduino.h>

#include <driver/adc.h>

class AdcSignal {
public:
    AdcSignal(adc1_channel_t channel);
    void init();
    uint16_t readRaw();
    void startConversion();
    uint16_t readResult();

private:
    adc1_channel_t _channel;
};

#endif // ADC_SIGNAL_H
