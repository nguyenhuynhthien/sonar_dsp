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
    volatile bool txEnabled;
    
    // Simulator config
    volatile uint32_t simDelaySamples;
    volatile bool simEnabled;
    
    volatile uint16_t servoAngle;
    volatile bool angleUpdated;
    volatile bool sweepDirectionCCW; // true = CCW (increasing), false = CW (decreasing)

    // Target information and streaming configuration
    volatile int32_t targetRange;
    volatile int32_t targetStrength;
    volatile bool targetDetected;
    volatile uint8_t streamMode; // 0: raw, 1: demod, 2: compressed
    volatile bool accumulatedDataReady;
    volatile bool requestServoStep;
    volatile bool stepComplete;

    // Waveform and velocity synchronization variables
    int16_t waveSendBuffer[Constant::ADC_SAMPLES];
    volatile uint16_t waveSendAngle;
    volatile bool waveSendReady;
    volatile int peakIndexForVelocity;
    volatile bool velocityRequested;
    volatile int pulseIndex;
    volatile uint32_t txPeriodMs;

    // Full 8x2048 Sum Matrix for 8 pulses (Slow-time complex IQ signal)
    // Allocated once at setup() in internal DRAM heap to prevent static BSS overflow
    int16_t* matrixSum_I[8];
    int16_t* matrixSum_Q[8];

    // Difference channel 8-pulse accumulator array & Sum channel 8-pulse accumulator array
    int16_t* diffAccumulator;
    int16_t* sumAccumulator;



    // Shared temporary demodulation buffers
    int16_t sharedDemodI[Constant::ADC_SAMPLES];
    int16_t sharedDemodQ[Constant::ADC_SAMPLES];

    // Single shared complex scratchpad buffer for ALL DSP operations (Matched Filter & FFT)
    float dsp_scratchpad[Constant::ADC_SAMPLES][2];


    // Shared variables for Sum-channel Peak detection and FFT
    int sharedPeakIdx;
    volatile int sharedWindowCenterIdx;
    int16_t sharedFftReal[Constant::DOPPLER_FFT_LEN];
    int16_t sharedFftImag[Constant::DOPPLER_FFT_LEN];
};


#endif // SHARED_SONAR_DATA_H
