#include "ScannerApp.h"
#include "TransmitterApp.h"
#include "../service/ComManager.h"

ScannerApp::ScannerApp(ServoService &servoService, SharedSonarData &sharedData, ComManager &com)
    : _servoService(servoService), _sharedData(sharedData), _com(com), _currentAngle(0),
      _step(Constant::SERVO_STEP_DEG) {}

void ScannerApp::begin() {
  _servoService.init(14); // GPIO14
  _servoService.writeAngle(_currentAngle);
}

void ScannerApp::step() {
  _currentAngle += _step;
  if (_currentAngle >= 180) {
    _currentAngle = 180;
    _step = -Constant::SERVO_STEP_DEG; // Reverse direction
  } else if (_currentAngle <= 0) {
    _currentAngle = 0;
    _step = Constant::SERVO_STEP_DEG; // Reverse direction
  }
  _servoService.writeAngle(_currentAngle);

  taskENTER_CRITICAL(&_sharedData.spinlock);
  _sharedData.servoAngle = _currentAngle;
  _sharedData.sweepDirectionCCW = (_step > 0);
  _sharedData.angleUpdated = true;
  taskEXIT_CRITICAL(&_sharedData.spinlock);

  uint16_t angleToSend = _currentAngle;
  if (_step <= 0) angleToSend |= 0x8000; // CW sweep direction flag
  _com.sendAngle(angleToSend);
}

void ScannerApp::run() { step(); }
