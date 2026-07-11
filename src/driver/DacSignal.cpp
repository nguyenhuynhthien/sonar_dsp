#include "DacSignal.h"
#include <driver/dac.h>
#include <Constant.hpp>

void DacSignal::init() {
    // Enable DAC channel 1 (GPIO 25)
    dac_output_enable(DAC_CHANNEL_1);
    // Write initial DC bias
    writeDCBias();
}

void DacSignal::writeSample(uint8_t value) {
    // Direct register or ESP-IDF write
    dac_output_voltage(DAC_CHANNEL_1, value);
}

void DacSignal::writeDCBias() {
    dac_output_voltage(DAC_CHANNEL_1, Constant::DAC_DC_BIAS);
}
