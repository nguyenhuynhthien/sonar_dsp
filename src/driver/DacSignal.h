#ifndef DAC_SIGNAL_H
#define DAC_SIGNAL_H

#include <Arduino.h>

#include <driver/dac.h>

class DacSignal {
public:
    DacSignal(dac_channel_t channel);
    void init();
    void writeSample(uint8_t value);
    void writeDCBias();

private:
    dac_channel_t _channel;
};

#endif // DAC_SIGNAL_H
