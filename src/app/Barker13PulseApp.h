#ifndef BARKER13_PULSE_APP_H
#define BARKER13_PULSE_APP_H

#include <Arduino.h>

class Barker13PulseApp {
public:
    static void generateBarker13(uint8_t* buffer, size_t size);
};

#endif // BARKER13_PULSE_APP_H
