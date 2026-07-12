#include "DacService.h"
#include "../driver/DacSignal.h"

DacService::DacService(dac_channel_t channel) : _dacSignal(channel) {
}

void DacService::init() {
    _dacSignal.init();
}

void DacService::writeSample(uint8_t value) {
    _dacSignal.writeSample(value);
}

void DacService::writeDCBias() {
    _dacSignal.writeDCBias();
}
