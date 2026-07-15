#ifndef SYNC_SIGNAL_SERVICE_H
#define SYNC_SIGNAL_SERVICE_H

#include <Arduino.h>
#include "DacService.h"
#include "../driver/AdcSignal.h"

class SyncSignalService {
public:
    static void init();
    static void sampleAndPlay(uint16_t* adcDestBuffer1, uint16_t* adcDestBuffer2, size_t size, 
                             const uint8_t* dac1SrcBuffer, const uint8_t* dac2SrcBuffer, 
                             DacService& dac1, DacService& dac2,
                             AdcSignal& adc1, AdcSignal& adc2);
};

#endif // SYNC_SIGNAL_SERVICE_H
