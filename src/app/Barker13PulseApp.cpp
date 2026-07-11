#include "Barker13PulseApp.h"
#include "../service/DacService.h"
#include <Constant.hpp>
#include <string.h>

void Barker13PulseApp::init() {
    DacService::init();
}

void Barker13PulseApp::writeSample(uint8_t value) {
    DacService::writeSample(value);
}

void Barker13PulseApp::writeDCBias() {
    DacService::writeDCBias();
}

void Barker13PulseApp::generateBarker13(uint8_t* buffer, size_t size) {
    // Fill the buffer with DAC DC bias as default
    memset(buffer, Constant::DAC_DC_BIAS, size);
 
    // Pulse length
    size_t pulse_len = Constant::BARKER13_PULSE_LEN;
    if (pulse_len > size) {
        pulse_len = size;
    }
 
    // Copy pre-calculated waveform directly from flash to save CPU cycles
    memcpy(buffer, Constant::BARKER13_PULSE_WAVE, pulse_len);
}
