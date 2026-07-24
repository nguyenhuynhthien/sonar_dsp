#include "TransmitterApp.h"
#include "SinglePulseApp.h"
#include "Barker13PulseApp.h"

TransmitterApp::TransmitterApp(ComManager& com, SharedSonarData& sharedData)
    : _com(com), _sharedData(sharedData), _frameId(0), _pulseType(ComManager::PULSE_SINGLE),
      _txPulseLen(Constant::FILTER_COEFFS_LEN), _txGain(1.0f), _txEnabled(false) {
}

void TransmitterApp::begin() {
    // Default is Tx disabled, so initialize txBuffer with pure bias level (127)
    taskENTER_CRITICAL(&_sharedData.spinlock);
    memset(_sharedData.txBuffer, Constant::DAC_DC_BIAS, Constant::BARKER13_PULSE_LEN);
    _sharedData.txPulseLen = Constant::FILTER_COEFFS_LEN;
    _sharedData.txEnabled = false;
    taskEXIT_CRITICAL(&_sharedData.spinlock);
}

void TransmitterApp::run() {
    _com.update(); // Poll UDP socket commands

    // Check if pulse type, gain, or tx enable changed in _com
    ComManager::PulseType currentType = _com.getPulseType();
    float currentGain = _com.getTxGain();
    bool currentTxEnabled = _com.isTxEnabled();
    if (currentType != _pulseType || currentGain != _txGain || currentTxEnabled != _txEnabled) {
        _pulseType = currentType;
        _txGain = currentGain;
        _txEnabled = currentTxEnabled;
        taskENTER_CRITICAL(&_sharedData.spinlock);
        _sharedData.txEnabled = _txEnabled;
        if (!_txEnabled) {
            memset(_sharedData.txBuffer, Constant::DAC_DC_BIAS, Constant::BARKER13_PULSE_LEN);
            _sharedData.txPulseLen = (_pulseType == ComManager::PULSE_SINGLE) ? Constant::FILTER_COEFFS_LEN : Constant::BARKER13_PULSE_LEN;
            _sharedData.txPeriodMs = (_pulseType == ComManager::PULSE_SINGLE) ? Constant::PRI_SINGLE_MS : Constant::PRI_BARKER13_MS;
            Serial.println("TransmitterApp: Tx Disabled (Pure Bias)");
        } else {
            if (_pulseType == ComManager::PULSE_SINGLE) {
                for (size_t i = 0; i < Constant::FILTER_COEFFS_LEN; ++i) {
                    int dev = (int)Constant::SINGLE_PULSE_WAVE[i] - Constant::DAC_DC_BIAS;
                    _sharedData.txBuffer[i] = (uint8_t)(Constant::DAC_DC_BIAS + (int)(dev * _txGain));
                }
                _sharedData.txPulseLen = Constant::FILTER_COEFFS_LEN;
                _sharedData.txPeriodMs = Constant::PRI_SINGLE_MS;
                Serial.printf("TransmitterApp: Single Pulse with gain %.4f\n", _txGain);
            } else {
                for (size_t i = 0; i < Constant::BARKER13_PULSE_LEN; ++i) {
                    int dev = (int)Constant::BARKER13_PULSE_WAVE[i] - Constant::DAC_DC_BIAS;
                    _sharedData.txBuffer[i] = (uint8_t)(Constant::DAC_DC_BIAS + (int)(dev * _txGain));
                }
                _sharedData.txPulseLen = Constant::BARKER13_PULSE_LEN;
                _sharedData.txPeriodMs = Constant::PRI_BARKER13_MS;
                Serial.printf("TransmitterApp: Barker 13 with gain %.4f\n", _txGain);
            }
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
            // Clear any pending notifications
            ulTaskNotifyTake(pdTRUE, 0);
            xTaskNotifyGive(_sharedData.rxTaskHandle);
        }

        // Wait for Core 1 using task notification (0% CPU, instant wakeup)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(Constant::TX_RESPONSE_TIMEOUT_MS));

        // Wait for ScannerTask to process the step and clear requestServoStep
        unsigned long waitStart2 = millis();
        while (_sharedData.requestServoStep && (millis() - waitStart2) < Constant::TX_RESPONSE_TIMEOUT_MS) {
            vTaskDelay(1); // Yield CPU to let the lower-priority ScannerTask run
        }

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
        }
    } else {
        // Idle state
        vTaskDelay(pdMS_TO_TICKS(Constant::TX_IDLE_DELAY_MS));
    }
}
