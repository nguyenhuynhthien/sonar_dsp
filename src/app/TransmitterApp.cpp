#include "TransmitterApp.h"
#include "SinglePulseApp.h"

// Inline assembly to read CPU cycles (ccount register)
static inline uint32_t get_ccount() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount" : "=r"(ccount));
    return ccount;
}

TransmitterApp::TransmitterApp(ComManager& com, SharedSonarData& sharedData)
    : _com(com), _sharedData(sharedData), _frameId(0) {
}

void TransmitterApp::begin() {
    SinglePulseApp::init();
    // Pre-generate single pulse wave
    SinglePulseApp::generateSinglePulse(_txBuffer, Constant::ADC_SAMPLES);
}

void TransmitterApp::run() {
    _com.update();

    if (_com.isStreaming()) {
        unsigned long startTime = millis();

        // 1. Trigger Receiver on Core 1
        taskENTER_CRITICAL(&_sharedData.spinlock);
        _sharedData.triggerTx = true;
        _sharedData.processingDone = false;
        taskEXIT_CRITICAL(&_sharedData.spinlock);

        // 2. Generate Pulse Burst via DAC1 (GPIO 25)
        uint32_t cpu_freq_mhz = ESP.getCpuFreqMHz();
        uint32_t cycles_per_sample = (uint32_t)(cpu_freq_mhz * Constant::CPU_CYCLES_PER_SAMPLE_FACTOR);
        uint32_t start_cycles = get_ccount();
        
        // Output the pulse burst
        for (size_t i = 0; i < Constant::FILTER_COEFFS_LEN; ++i) {
            uint32_t target_cycles = start_cycles + i * cycles_per_sample;
            while (get_ccount() < target_cycles) {
                // Precision wait
            }
            SinglePulseApp::writeSample(_txBuffer[i]);
        }

        // CRITICAL BUG FIX 1: Immediately restore and maintain 1.65V steady DC bias (value 127)
        SinglePulseApp::writeDCBias();

        // 3. Wait for Core 1 (ReceiverApp) to finish sampling and DSP processing
        uint32_t timeoutTicks = pdMS_TO_TICKS(Constant::TX_RESPONSE_TIMEOUT_MS);
        uint32_t elapsedTicks = 0;
        while (!_sharedData.processingDone && elapsedTicks < timeoutTicks) {
            vTaskDelay(1);
            elapsedTicks++;
        }

        // 4. Retrieve sampled buffer and stream via UDP
        if (_sharedData.processingDone) {
            taskENTER_CRITICAL(&_sharedData.spinlock);
            memcpy(_localAdcBuffer, (const void*)_sharedData.adcBuffer, sizeof(_localAdcBuffer));
            _sharedData.processingDone = false;
            taskEXIT_CRITICAL(&_sharedData.spinlock);

            _com.sendFrame(_frameId++, _localAdcBuffer, Constant::ADC_SAMPLES);
            // Serial.printf("Frame %d sent successfully via UDP!\n", _frameId - 1);
        } else {
            Serial.println("Error: Timeout waiting for Core 1 to complete sampling!");
        }

        // Maintain configured rate/period
        unsigned long duration = millis() - startTime;
        if (duration < Constant::TX_PERIOD_MS) {
            vTaskDelay(pdMS_TO_TICKS(Constant::TX_PERIOD_MS - duration));
        }
    } else {
        // Idle state: force steady DC offset to keep ADC baseline stable
        SinglePulseApp::writeDCBias();
        vTaskDelay(pdMS_TO_TICKS(Constant::TX_IDLE_DELAY_MS));
    }
}
