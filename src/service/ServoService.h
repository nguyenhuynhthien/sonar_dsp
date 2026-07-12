#ifndef SERVO_SERVICE_H
#define SERVO_SERVICE_H

#include <Arduino.h>
#include <ESP32Servo.h>

class ServoService {
public:
    ServoService();
    void init(int pin);
    void writeAngle(int angle);
    int readAngle();
private:
    Servo _servo;
    int _pin;
};

#endif // SERVO_SERVICE_H
