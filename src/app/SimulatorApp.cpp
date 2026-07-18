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

// Gaussian noise generator using Box-Muller transform (sigma = 7.0)
static int generateGaussianNoise(float sigma) {
    float u1 = ((float)esp_random() + 1.0f) / 4294967297.0f; // range (0, 1)
    float u2 = (float)esp_random() / 4294967295.0f; // range [0, 1]
    float z = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
    return (int)roundf(z * sigma);
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
        { 45.0f,  400, 0.9f, 10.0f },
        { 90.0f,  850, 0.7f, 10.0f },
        { 135.0f, 700, 0.7f, 10.0f }
    };

    // Find if the current angle is within any target's beamwidth
    int activeIndex = -1;
    float activeBeamScale = 0.0f;
    for (int i = 0; i < 3; ++i) {
        float diff = abs((float)currentAngle - targets[i].angle);
        if (diff < targets[i].beamwidth) {
            activeIndex = i;
            // Triangular beam pattern response: peak at center, 0 at beamwidth edge
            activeBeamScale = (1.0f - (diff / targets[i].beamwidth)) * targets[i].magnitude;
            break; // Assuming non-overlapping targets
        }
    }

    // 2. Fill the buffer with ambient noise and insert the simulated echo if target is active
    uint32_t simDelay = (activeIndex != -1) ? targets[activeIndex].delay : 0;

    // Doppler phase shift parameters for simulated moving targets
    float cosTheta = 1.0f;
    float sinTheta = 0.0f;
    if (activeIndex != -1) {
        float fd = 0.0f;
        if (activeIndex == 0) {
            fd = -20.83f;  // Bin -4 -> v = +0.09 m/s
        } else if (activeIndex == 1) {
            fd = 10.42f;   // Bin +2 -> v = -0.045 m/s
        } else if (activeIndex == 2) {
            fd = -10.42f;  // Bin -2 -> v = +0.045 m/s
        }
        
        int p = 0;
        taskENTER_CRITICAL(&_sharedData.spinlock);
        p = _sharedData.pulseIndex;
        taskEXIT_CRITICAL(&_sharedData.spinlock);
        
        for (size_t i = 0; i < size; ++i) {
            // Generate a shared random noise component for consistency
            int noise = generateGaussianNoise(7.0f);

            if (activeIndex != -1 && i >= simDelay && i < simDelay + actualPulseLen) {
                size_t pulseIdx = i - simDelay;
                
                float fs = (float)Constant::SAMPLE_RATE;
                float PRI = (float)_sharedData.txPeriodMs / 1000.0f;
                
                // Absolute time: t = time within the ADC buffer
                float t = (float)i / fs;
                float dopplerPhase = 2.0f * M_PI * fd * (t + (float)p * PRI) + (M_PI / 6.0f);
                
                // Single Sideband (SSB) modulation of the exact transmitted pulse waveform
                float baseDev = (float)localPulse[pulseIdx] - (float)Constant::DAC_DC_BIAS;
                float baseDevPrev = (pulseIdx > 0) ? ((float)localPulse[pulseIdx - 1] - (float)Constant::DAC_DC_BIAS) : 0.0f;
                float distortedDeviation = (baseDev * cosf(dopplerPhase) - baseDevPrev * sinf(dopplerPhase)) * activeBeamScale;

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
    } else {
        // No target active
        for (size_t i = 0; i < size; ++i) {
            int noise = generateGaussianNoise(7.0f);
            int simVal = (int)Constant::DAC_DC_BIAS + noise;
            if (simVal > 255) simVal = 255;
            if (simVal < 0) simVal = 0;
            buffer[i] = (uint8_t)simVal;
        }
    }
}

