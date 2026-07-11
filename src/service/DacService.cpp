#include "DacService.h"
#include "../driver/DacSignal.h"

void DacService::init() {
    DacSignal::init();
}

void DacService::writeSample(uint8_t value) {
    DacSignal::writeSample(value);
}

void DacService::writeDCBias() {
    DacSignal::writeDCBias();
}
