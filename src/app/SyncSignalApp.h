#ifndef SYNC_SIGNAL_APP_H
#define SYNC_SIGNAL_APP_H

#include <Arduino.h>
#include "TransmitterApp.h"
#include "SimulatorApp.h"
#include "../service/DacService.h"
#include "../driver/AdcSignal.h"

class SyncSignalApp {
public:
    SyncSignalApp(SharedSonarData& sharedData, DacService& dac1, DacService& dac2, SimulatorApp& simulator, AdcSignal& adc1, AdcSignal& adc2);
    void begin();
    void run();

private:
    SharedSonarData& _sharedData;
    DacService& _dac1;
    DacService& _dac2;
    SimulatorApp& _simulator;
    AdcSignal& _adc1;
    AdcSignal& _adc2;
};

#endif // SYNC_SIGNAL_APP_H
