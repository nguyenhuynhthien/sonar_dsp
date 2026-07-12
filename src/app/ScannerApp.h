#ifndef SCANNER_APP_H
#define SCANNER_APP_H

#include <Arduino.h>
#include "../service/ServoService.h"

struct SharedSonarData;

class ScannerApp {
public:
    ScannerApp(ServoService& servoService, SharedSonarData& sharedData);
    void begin();
    void step();
    void run();

private:
    ServoService& _servoService;
    SharedSonarData& _sharedData;
    int _currentAngle;
    int _step;
};

#endif // SCANNER_APP_H
