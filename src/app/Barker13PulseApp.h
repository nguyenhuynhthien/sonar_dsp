#ifndef BARKER13_PULSE_APP_H
#define BARKER13_PULSE_APP_H

#include <Arduino.h>

class Barker13PulseApp {
public:
    static void init();
    static void generateBarker13(uint8_t* buffer, size_t size);
    static void writeSample(uint8_t value);
    static void writeDCBias();
};

#endif // BARKER13_PULSE_APP_H
