#include "DacSignal.h"
#include <driver/dac.h>
#include <Constant.hpp>

DacSignal::DacSignal(dac_channel_t channel) : _channel(channel) {
}

void DacSignal::init() {
    // Enable the specified DAC channel
    dac_output_enable(_channel);
    // Write initial DC bias
    writeDCBias();
}

void DacSignal::writeSample(uint8_t value) {
    dac_output_voltage(_channel, value);
}

void DacSignal::writeDCBias() {
    dac_output_voltage(_channel, Constant::DAC_DC_BIAS);
}
