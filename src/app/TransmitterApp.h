#ifndef TRANSMITTER_APP_H
#define TRANSMITTER_APP_H

#include <Arduino.h>
#include "../service/ComManager.h"
#include <Constant.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ScannerApp.h"

struct SharedSonarData {
    volatile bool triggerTx;
    volatile bool processingDone;
    volatile bool adcReady;
    uint16_t adcBuffer[Constant::ADC_SAMPLES];
    portMUX_TYPE spinlock;
    TaskHandle_t rxTaskHandle;
    TaskHandle_t servoTaskHandle;
    
    // Shared transmit pulse configuration
    uint8_t txBuffer[Constant::BARKER13_PULSE_LEN];
    volatile size_t txPulseLen;
    
    volatile uint16_t servoAngle;
    volatile bool angleUpdated;
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
    ComManager::PulseType _pulseType;
    size_t _txPulseLen;
};

#endif // TRANSMITTER_APP_H
