#ifndef TRANSMITTER_APP_H
#define TRANSMITTER_APP_H

#include <Arduino.h>
#include "../service/ComManager.h"
#include <Constant.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

struct SharedSonarData {
    volatile bool triggerTx;
    volatile bool processingDone;
    uint16_t adcBuffer[Constant::ADC_SAMPLES];
    portMUX_TYPE spinlock;
};

class TransmitterApp {
public:
    TransmitterApp(ComManager& com, SharedSonarData& sharedData);
    void begin();
    void run();

private:
    ComManager& _com;
    SharedSonarData& _sharedData;
    uint8_t _txBuffer[Constant::ADC_SAMPLES];
    uint16_t _localAdcBuffer[Constant::ADC_SAMPLES];
    uint16_t _frameId;
};

#endif // TRANSMITTER_APP_H
