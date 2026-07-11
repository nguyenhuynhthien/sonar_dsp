#ifndef DAC_SIGNAL_H
#define DAC_SIGNAL_H

#include <Arduino.h>

class DacSignal {
public:
    static void init();
    static void writeSample(uint8_t value);
    static void writeDCBias();
};

#endif // DAC_SIGNAL_H
