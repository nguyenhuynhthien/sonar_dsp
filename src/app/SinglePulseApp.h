#ifndef SINGLE_PULSE_APP_H
#define SINGLE_PULSE_APP_H

#include <Arduino.h>

class SinglePulseApp {
public:
    static void init();
    static void generateSinglePulse(uint8_t* buffer, size_t size);
    static void generateBarker13(uint8_t* buffer, size_t size);
    static void writeSample(uint8_t value);
    static void writeDCBias();
};

#endif // SINGLE_PULSE_APP_H
