#ifndef SIMULATOR_APP_H
#define SIMULATOR_APP_H

#include <Arduino.h>
#include "TransmitterApp.h"

class SimulatorApp {
public:
    SimulatorApp(SharedSonarData& sharedData);
    void begin();
    void setEnabled(bool enabled);
    void setDelaySamples(uint32_t samples);
    bool isEnabled() const;
    uint32_t getDelaySamples() const;
    void fillSimulatorBuffer(uint8_t* buffer, size_t size, const uint8_t* txBuffer, size_t txPulseLen);

private:
    SharedSonarData& _sharedData;
};

#endif // SIMULATOR_APP_H
