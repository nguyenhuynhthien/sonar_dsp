#ifndef DAC_SERVICE_H
#define DAC_SERVICE_H

#include <Arduino.h>

#include <driver/dac.h>
#include "../driver/DacSignal.h"

class DacService {
public:
    DacService(dac_channel_t channel);
    void init();
    void writeSample(uint8_t value);
    void writeDCBias();

private:
    DacSignal _dacSignal;
};

#endif // DAC_SERVICE_H
