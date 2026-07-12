#include "ServoService.h"

ServoService::ServoService() : _pin(-1) {}

void ServoService::init(int pin) {
    _pin = pin;
    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    _servo.setPeriodHertz(50); // Standard 50Hz servo
    _servo.attach(_pin, 500, 2400); // MG90S typically min/max pulse width (500us to 2400us)
}

void ServoService::writeAngle(int angle) {
    if (_pin != -1) {
        _servo.write(angle);
    }
}

int ServoService::readAngle() {
    if (_pin != -1) {
        return _servo.read();
    }
    return 0;
}
