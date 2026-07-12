#ifndef SINGLE_PULSE_APP_H
#define SINGLE_PULSE_APP_H

#include <Arduino.h>

class SinglePulseApp {
public:
  static void generateSinglePulse(uint8_t *buffer, size_t size);
};

#endif // SINGLE_PULSE_APP_H
