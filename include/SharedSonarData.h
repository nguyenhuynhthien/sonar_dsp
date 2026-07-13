#ifndef SHARED_SONAR_DATA_H
#define SHARED_SONAR_DATA_H

#include <Arduino.h>
#include <Constant.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

struct SharedSonarData {
    volatile bool triggerTx;
    volatile bool processingDone;
    volatile bool adcReady;
    uint16_t adcBuffer[Constant::ADC_SAMPLES];
    portMUX_TYPE spinlock;
    TaskHandle_t rxTaskHandle;
    TaskHandle_t servoTaskHandle;
    TaskHandle_t rxCore0TaskHandle;
    TaskHandle_t waveSendTaskHandle;
    
    // Shared transmit pulse configuration
    uint8_t txBuffer[Constant::BARKER13_PULSE_LEN];
    volatile size_t txPulseLen;
    
    // Simulator config
    volatile uint32_t simDelaySamples;
    volatile bool simEnabled;
    
    volatile uint16_t servoAngle;
    volatile bool angleUpdated;

    // Target information and streaming configuration
    volatile float targetRange;
    volatile float targetStrength;
    volatile bool targetDetected;
    volatile uint8_t streamMode; // 0: raw, 1: demod, 2: compressed
    volatile bool accumulatedDataReady;
    volatile bool requestServoStep;

    // Waveform and velocity synchronization variables
    uint16_t waveSendBuffer[Constant::ADC_SAMPLES];
    volatile uint16_t waveSendAngle;
    volatile bool waveSendReady;
    volatile int peakIndexForVelocity;
    volatile bool velocityRequested;
    volatile int pulseIndex;
};

#endif // SHARED_SONAR_DATA_H
