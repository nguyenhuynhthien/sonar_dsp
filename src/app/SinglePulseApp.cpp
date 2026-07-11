#include "SinglePulseApp.h"
#include "../service/DacService.h"
#include <Constant.hpp>
#include <math.h>

void SinglePulseApp::init() { DacService::init(); }

void SinglePulseApp::writeSample(uint8_t value) {
  DacService::writeSample(value);
}

void SinglePulseApp::writeDCBias() { DacService::writeDCBias(); }

void SinglePulseApp::generateSinglePulse(uint8_t *buffer, size_t size) {
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