#include "SyncSignalApp.h"
#include "../service/SyncSignalService.h"
#include <Constant.hpp>

SyncSignalApp::SyncSignalApp(SharedSonarData& sharedData, DacService& dac1, DacService& dac2, SimulatorApp& simulator)
    : _sharedData(sharedData), _dac1(dac1), _dac2(dac2), _simulator(simulator) {
}

void SyncSignalApp::begin() {
    SyncSignalService::init();
    _dac1.init();
    _dac2.init();
}

void SyncSignalApp::run() {
    // 1. Wait indefinitely for a task notification (trigger signal) from Core 0
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Reset trigger Tx flag
    taskENTER_CRITICAL(&_sharedData.spinlock);
    _sharedData.triggerTx = false;
    taskEXIT_CRITICAL(&_sharedData.spinlock);

    // 2. Prepare DAC1 original transmit pulse buffer
    static uint8_t dac1Buffer[Constant::ADC_SAMPLES];
    memset(dac1Buffer, Constant::DAC_DC_BIAS, sizeof(dac1Buffer));
    
    taskENTER_CRITICAL(&_sharedData.spinlock);
    size_t pulseLen = _sharedData.txPulseLen;
    memcpy(dac1Buffer, (const void*)_sharedData.txBuffer, pulseLen);
    taskEXIT_CRITICAL(&_sharedData.spinlock);

    // 3. Prepare DAC2 simulated target echo buffer via SimulatorApp
    static uint8_t dac2Buffer[Constant::ADC_SAMPLES];
    _simulator.fillSimulatorBuffer(dac2Buffer, Constant::ADC_SAMPLES, dac1Buffer, pulseLen);

    // 4. Inform TransmitterApp/Core 0 that ADC is starting (simulate adcReady)
    taskENTER_CRITICAL(&_sharedData.spinlock);
    _sharedData.adcReady = true;
    taskEXIT_CRITICAL(&_sharedData.spinlock);

    // 5. Trigger hardware synchronization loop
    SyncSignalService::sampleAndPlay((uint16_t*)_sharedData.adcBuffer, Constant::ADC_SAMPLES, dac1Buffer, dac2Buffer, _dac1, _dac2);
}
