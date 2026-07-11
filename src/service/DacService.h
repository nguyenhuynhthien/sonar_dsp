#ifndef DAC_SERVICE_H
#define DAC_SERVICE_H

#include <Arduino.h>

class DacService {
public:
    static void init();
    static void writeSample(uint8_t value);
    static void writeDCBias();
};

#endif // DAC_SERVICE_H
