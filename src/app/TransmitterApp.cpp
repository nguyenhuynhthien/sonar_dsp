#include "TransmitterApp.h"
#include "SinglePulseApp.h"
#include "Barker13PulseApp.h"

TransmitterApp::TransmitterApp(ComManager& com, SharedSonarData& sharedData)
    : _com(com), _sharedData(sharedData), _frameId(0), _pulseType(ComManager::PULSE_SINGLE), _txPulseLen(Constant::FILTER_COEFFS_LEN) {
}

void TransmitterApp::begin() {
    // Copy default single pulse wave to shared memory
    taskENTER_CRITICAL(&_sharedData.spinlock);
    memcpy(_sharedData.txBuffer, Constant::SINGLE_PULSE_WAVE, Constant::FILTER_COEFFS_LEN);
    _sharedData.txPulseLen = Constant::FILTER_COEFFS_LEN;
    taskEXIT_CRITICAL(&_sharedData.spinlock);
}

void TransmitterApp::run() {
    _com.update(); // Poll UDP socket commands

    // Check if pulse type changed in _com
    ComManager::PulseType currentType = _com.getPulseType();
    if (currentType != _pulseType) {
        _pulseType = currentType;
        taskENTER_CRITICAL(&_sharedData.spinlock);
        if (_pulseType == ComManager::PULSE_SINGLE) {
            memcpy(_sharedData.txBuffer, Constant::SINGLE_PULSE_WAVE, Constant::FILTER_COEFFS_LEN);
            _sharedData.txPulseLen = Constant::FILTER_COEFFS_LEN;
            _sharedData.txPeriodMs = Constant::PRI_SINGLE_MS;
            Serial.println("TransmitterApp: switched to Single Pulse");
        } else {
            memcpy(_sharedData.txBuffer, Constant::BARKER13_PULSE_WAVE, Constant::BARKER13_PULSE_LEN);
            _sharedData.txPulseLen = Constant::BARKER13_PULSE_LEN;
            _sharedData.txPeriodMs = Constant::PRI_BARKER13_MS;
            Serial.println("TransmitterApp: switched to Barker 13");
        }
        taskEXIT_CRITICAL(&_sharedData.spinlock);
    }

    if (_com.isStreaming()) {
        unsigned long startMicros = micros();

        int currentPulse = 0;
        taskENTER_CRITICAL(&_sharedData.spinlock);
        currentPulse = _sharedData.pulseIndex;
        if (currentPulse == 0) {
            _sharedData.stepComplete = false;
        }
        _sharedData.triggerTx = true;
        _sharedData.processingDone = 0;
        _sharedData.adcReady = false;
        _sharedData.streamMode = (uint8_t)_com.getStreamMode();
        taskEXIT_CRITICAL(&_sharedData.spinlock);

        if (_sharedData.rxTaskHandle != nullptr) {
            xTaskNotifyGive(_sharedData.rxTaskHandle);
        }

        // 2. Wait for Core 1: on Pulse 7, wait for the entire step to complete, otherwise wait for Rx1/Rx2
        unsigned long waitStart = millis();
        if (currentPulse == 7) {
            while (!_sharedData.stepComplete && (millis() - waitStart) < Constant::TX_RESPONSE_TIMEOUT_MS) {
                vTaskDelay(1);
            }
        } else {
            while (_sharedData.processingDone != 3 && (millis() - waitStart) < Constant::TX_RESPONSE_TIMEOUT_MS) {
                vTaskDelay(1);
            }
        }
        unsigned long waitRxTime = millis() - waitStart;

        // Wait for ScannerTask to process the step and clear requestServoStep
        unsigned long waitStart2 = millis();
        while (_sharedData.requestServoStep && (millis() - waitStart2) < Constant::TX_RESPONSE_TIMEOUT_MS) {
            vTaskDelay(1); // Yield CPU to let the lower-priority ScannerTask run
        }
        unsigned long waitServoTime = millis() - waitStart2;



        // Clear processingDone for the next cycle
        taskENTER_CRITICAL(&_sharedData.spinlock);
        _sharedData.processingDone = 0;
        taskEXIT_CRITICAL(&_sharedData.spinlock);

        // Maintain configured rate/period (PRI) using hybrid microsecond-accurate timer
        uint32_t periodMs = 30;
        taskENTER_CRITICAL(&_sharedData.spinlock);
        periodMs = _sharedData.txPeriodMs;
        taskEXIT_CRITICAL(&_sharedData.spinlock);

        unsigned long elapsedUs = micros() - startMicros;
        unsigned long periodUs = periodMs * 1000;
        if (elapsedUs < periodUs) {
            unsigned long remainingUs = periodUs - elapsedUs;
            if (remainingUs > Constant::TX_YIELD_THRESHOLD_US) {
                // Yield to other tasks for the bulk of the remaining time
                vTaskDelay(pdMS_TO_TICKS(remainingUs / 1000 - 1));
            }
            // Busy-wait for precise microsecond alignment
            while ((micros() - startMicros) < periodUs) {
                delayMicroseconds(10);
            }
        } else {
            // Force 1 tick delay to prevent task watchdog starvation on Core 0
            vTaskDelay(1);
        }
    } else {
        // Idle state
        vTaskDelay(pdMS_TO_TICKS(Constant::TX_IDLE_DELAY_MS));
    }
}
