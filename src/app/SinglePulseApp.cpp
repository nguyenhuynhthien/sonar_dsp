#include "SinglePulseApp.h"
#include "../service/DacService.h"
#include <Constant.hpp>
#include <math.h>

void SinglePulseApp::init() {
    DacService::init();
}

void SinglePulseApp::writeSample(uint8_t value) {
    DacService::writeSample(value);
}

void SinglePulseApp::writeDCBias() {
    DacService::writeDCBias();
}

void SinglePulseApp::generateSinglePulse(uint8_t* buffer, size_t size) {
    // Fill the buffer with DAC DC bias as default
    memset(buffer, Constant::DAC_DC_BIAS, size);
 
    // Pulse length
    size_t pulse_len = Constant::FILTER_COEFFS_LEN;
    if (pulse_len > size) {
        pulse_len = size;
    }
 
    // Copy pre-calculated waveform directly from flash to save CPU cycles
    memcpy(buffer, Constant::SINGLE_PULSE_WAVE, pulse_len);
}

void SinglePulseApp::generateBarker13(uint8_t* buffer, size_t size) {
    // Fill buffer with DC bias
    memset(buffer, Constant::DAC_DC_BIAS, size);
 
    // Barker 13 sequence: +1, +1, +1, +1, +1, -1, -1, +1, +1, -1, +1, -1, +1
    int8_t barker[] = {1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1};
    size_t num_chips = 13;
    
    // Each chip is 1 cycle (4 samples at 160 kHz) to keep the total length small
    size_t samples_per_chip = 4; 
    size_t total_pulse_len = num_chips * samples_per_chip; // 52 samples
 
    if (total_pulse_len > size) {
        total_pulse_len = size;
    }
 
    for (size_t i = 0; i < total_pulse_len; ++i) {
        size_t chip_idx = i / samples_per_chip;
        int8_t phase_coeff = barker[chip_idx];
        double angle = (2.0 * M_PI * Constant::CENTER_FREQ * i) / Constant::SAMPLE_RATE;
        buffer[i] = (uint8_t)(Constant::DAC_DC_BIAS + Constant::DAC_DC_BIAS * phase_coeff * sin(angle));
    }
}
