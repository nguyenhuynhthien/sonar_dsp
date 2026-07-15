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
            fd = 23.32f;  // v = 0.10 m/s (Detectable at 15ms PRI)
        } else if (activeIndex == 1) {
            fd = -16.32f; // v = -0.07 m/s
        } else if (activeIndex == 2) {
            fd = 9.33f;   // v = 0.04 m/s
        }
        
        int p = 0;
        taskENTER_CRITICAL(&_sharedData.spinlock);
        p = _sharedData.pulseIndex;
        taskEXIT_CRITICAL(&_sharedData.spinlock);
        
        float theta = 2.0f * M_PI * fd * p * ((float)Constant::TX_PERIOD_MS / 1000.0f);
        
        for (size_t i = 0; i < size; ++i) {
            // Generate a shared random noise component for consistency
            int noise = ((int)(esp_random() % 31)) - 15; // [-15, 15]

            if (activeIndex != -1 && i >= simDelay && i < simDelay + actualPulseLen) {
                size_t pulseIdx = i - simDelay;
                
                float carrier = 0.0f;
                float sign = 1.0f;
                if (actualPulseLen == Constant::BARKER13_PULSE_LEN) {
                    static const float chips[13] = {1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1};
                    int chipIdx = pulseIdx / 8;
                    if (chipIdx >= 0 && chipIdx < 13) {
                        sign = chips[chipIdx];
                    }
                }
                
                // Generate 40kHz signal sampled at 160kHz
                // Use cosine for consistency with Real/Imag demodulation
                // The receiver samples at 0, 90, 180, 270 relative to the 40kHz reference.
                // We must use fc + fd to simulate the real physics of a moving target.
                float fc = (float)Constant::CENTER_FREQ;
                float fs = (float)Constant::SAMPLE_RATE;
                float PRI = (float)Constant::TX_PERIOD_MS / 1000.0f;
                
                // Signal: cos(2*pi*(fc + fd)*t + 2*pi*fc*deltaT_p)
                // where deltaT_p is the time delay change between pings.
                // However, the simplest correct simulation is:
                // Phase(p, i) = 2*pi*fc*(i/fs) + 2*pi*fd*(i/fs + p*PRI)
                float t = (float)i / fs;
                float totalPhase = 2.0f * M_PI * (fc * t + fd * ((float)p * PRI)) + (M_PI / 6.0f);
                carrier = sign * cosf(totalPhase);
                
                // Scale deviation with the beam scale
                float distortedDeviation = carrier * 127.0f * activeBeamScale;

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
            int noise = ((int)(esp_random() % 31)) - 15;
            int simVal = (int)Constant::DAC_DC_BIAS + noise;
            if (simVal > 255) simVal = 255;
            if (simVal < 0) simVal = 0;
            buffer[i] = (uint8_t)simVal;
        }
    }
}

