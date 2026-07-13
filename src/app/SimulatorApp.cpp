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

    // Get current servo angle
    int currentAngle = 0;
    taskENTER_CRITICAL(&_sharedData.spinlock);
    currentAngle = _sharedData.servoAngle;
    taskEXIT_CRITICAL(&_sharedData.spinlock);

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

    // Define 3 distinct targets
    struct Target {
        float angle;       // Center angle in degrees
        uint32_t delay;    // Delay in samples (cự li)
        float magnitude;   // Amplitude/gain scale (độ lớn)
        float beamwidth;   // Beam half-width in degrees
    };

    Target targets[3] = {
        { 45.0f,  400, 0.8f, 6.0f },
        { 90.0f,  850, 0.5f, 6.0f },
        { 135.0f, 1300, 0.4f, 6.0f }
    };

    // Find if the current angle is within any target's beamwidth
    const Target* activeTarget = nullptr;
    float activeBeamScale = 0.0f;
    for (int i = 0; i < 3; ++i) {
        float diff = abs((float)currentAngle - targets[i].angle);
        if (diff < targets[i].beamwidth) {
            activeTarget = &targets[i];
            // Triangular beam pattern response: peak at center, 0 at beamwidth edge
            activeBeamScale = (1.0f - (diff / targets[i].beamwidth)) * targets[i].magnitude;
            break; // Assuming non-overlapping targets
        }
    }

    // 2. Fill the buffer with ambient noise and insert the simulated echo if target is active
    uint32_t simDelay = activeTarget ? activeTarget->delay : 0;

    for (size_t i = 0; i < size; ++i) {
        // Generate a shared random noise component for consistency
        int noise = ((int)(esp_random() % 31)) - 15; // [-15, 15]

        if (activeTarget && i >= simDelay && i < simDelay + actualPulseLen) {
            size_t pulseIdx = i - simDelay;
            int deviation = (int)localPulse[pulseIdx] - (int)Constant::DAC_DC_BIAS;

            // Scale deviation with the beam scale
            float distortedDeviation = (float)deviation * activeBeamScale;

            int simVal = (int)Constant::DAC_DC_BIAS + (int)distortedDeviation + noise;

            if (simVal > 255) simVal = 255;
            if (simVal < 0) simVal = 0;

            buffer[i] = (uint8_t)simVal;
        } else {
            // Background ambient noise when no target is present
            int simVal = (int)Constant::DAC_DC_BIAS + noise;
            
            if (simVal > 255) simVal = 255;
            if (simVal < 0) simVal = 0;

            buffer[i] = (uint8_t)simVal;
        }
    }
}

