#include "SimulatorApp.h"
#include "SinglePulseApp.h"
#include "Barker13PulseApp.h"

SimulatorApp::SimulatorApp(SharedSonarData& sharedData) : _sharedData(sharedData) {
}

void SimulatorApp::begin() {
    // Simulator initialization (pins, default state)
}

void SimulatorApp::setEnabled(bool enabled) {
    taskENTER_CRITICAL(&_sharedData.spinlock);
    _sharedData.simEnabled = enabled;
    taskEXIT_CRITICAL(&_sharedData.spinlock);
}

void SimulatorApp::setDelaySamples(uint32_t samples) {
    taskENTER_CRITICAL(&_sharedData.spinlock);
    _sharedData.simDelaySamples = samples;
    taskEXIT_CRITICAL(&_sharedData.spinlock);
}

bool SimulatorApp::isEnabled() const {
    return _sharedData.simEnabled;
}

uint32_t SimulatorApp::getDelaySamples() const {
    return _sharedData.simDelaySamples;
}

void SimulatorApp::fillSimulatorBuffer(uint8_t* buffer, size_t size, const uint8_t* txBuffer, size_t txPulseLen) {
    if (!_sharedData.simEnabled) {
        // If simulator is disabled, fill completely with pure bias level
        memset(buffer, Constant::DAC_DC_BIAS, size);
        return;
    }

    // 1. Generate the raw pulse waveform using SinglePulseApp or Barker13PulseApp dynamically
    // based on the pulse length configured in sharedData.
    // If pulse length matches Barker13 length, generate Barker13, otherwise Single Pulse.
    uint8_t localPulse[Constant::BARKER13_PULSE_LEN];
    size_t actualPulseLen = txPulseLen;
    if (actualPulseLen == Constant::BARKER13_PULSE_LEN) {
        Barker13PulseApp::generateBarker13(localPulse, Constant::BARKER13_PULSE_LEN);
    } else {
        SinglePulseApp::generateSinglePulse(localPulse, Constant::FILTER_COEFFS_LEN);
        actualPulseLen = Constant::FILTER_COEFFS_LEN;
    }

    uint32_t simDelay = _sharedData.simDelaySamples;

    // 2. Fill the buffer with ambient noise and insert the simulated echo
    for (size_t i = 0; i < size; ++i) {
        if (i >= simDelay && i < simDelay + actualPulseLen) {
            size_t pulseIdx = i - simDelay;
            int deviation = (int)localPulse[pulseIdx] - (int)Constant::DAC_DC_BIAS;

            // Apply envelope distortion (fading) before and after the pulse center
            // Keep uniform envelope to preserve matched filter coding performance
            float envelope = 1.0f;

            // Scale deviation with the envelope and distance attenuation (0.5)
            float distortedDeviation = (float)deviation * envelope * 0.5f;

            // Add pseudo-random high frequency noise
            int noise = ((int)(esp_random() % 11)) - 5; // [-5, 5]

            int simVal = (int)Constant::DAC_DC_BIAS + (int)distortedDeviation + noise;

            if (simVal > 255) simVal = 255;
            if (simVal < 0) simVal = 0;

            buffer[i] = (uint8_t)simVal;
        } else {
            // Background ambient noise when no target is present
            int ambientNoise = ((int)(esp_random() % 5)) - 2; // [-2, 2]
            int simVal = (int)Constant::DAC_DC_BIAS + ambientNoise;
            
            if (simVal > 255) simVal = 255;
            if (simVal < 0) simVal = 0;

            buffer[i] = (uint8_t)simVal;
        }
    }
}
