#ifndef SHARED_SONAR_DATA_H
#define SHARED_SONAR_DATA_H

#include <Arduino.h>
#include <Constant.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

struct SharedSonarData {
    volatile bool triggerTx;
    volatile uint8_t processingDone;
    volatile bool adcReady;
    uint16_t adcBuffer1[Constant::ADC_SAMPLES];
    uint16_t adcBuffer2[Constant::ADC_SAMPLES];
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
    volatile int32_t targetRange;
    volatile int32_t targetStrength;
    volatile bool targetDetected;
    volatile uint8_t streamMode; // 0: raw, 1: demod, 2: compressed
    volatile bool accumulatedDataReady;
    volatile bool requestServoStep;

    // Waveform and velocity synchronization variables
    int16_t waveSendBuffer[Constant::ADC_SAMPLES];
    volatile uint16_t waveSendAngle;
    volatile bool waveSendReady;
    volatile int peakIndexForVelocity;
    volatile bool velocityRequested;
    volatile int pulseIndex;
    volatile uint32_t txPeriodMs;

    // Channel Matrices for Left (L) and Right (R) complex signals (Q15: real and imag)
    // Dimension: [8][2048] - Heap allocated to save static DRAM
    int16_t* channelL_I[8];
    int16_t* channelL_Q[8];
    int16_t* channelR_I[8];
    int16_t* channelR_Q[8];

    // Shared temporary buffers to avoid redundant heap allocations in ReceiverApps
    int16_t* sharedDemodI;
    int16_t* sharedDemodQ;

    // Shared variables for Sum-channel Peak detection and FFT
    int sharedPeakIdx;
    int16_t sharedFftReal[Constant::DOPPLER_FFT_LEN];
    int16_t sharedFftImag[Constant::DOPPLER_FFT_LEN];
};

#endif // SHARED_SONAR_DATA_H
