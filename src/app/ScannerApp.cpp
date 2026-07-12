#include "ScannerApp.h"
#include "TransmitterApp.h"

ScannerApp::ScannerApp(ServoService &servoService, SharedSonarData &sharedData)
    : _servoService(servoService), _sharedData(sharedData), _currentAngle(0), _step(3) {}

void ScannerApp::begin() {
  _servoService.init(14); // GPIO14
  _servoService.writeAngle(_currentAngle);
}

void ScannerApp::step() {
  _currentAngle += _step;
  if (_currentAngle >= 180) {
    _currentAngle = 180;
    _step = -3; // Reverse direction
  } else if (_currentAngle <= 0) {
    _currentAngle = 0;
    _step = 3; // Reverse direction
  }
  _servoService.writeAngle(_currentAngle);

  taskENTER_CRITICAL(&_sharedData.spinlock);
  _sharedData.servoAngle = _currentAngle;
  _sharedData.angleUpdated = true;
  taskEXIT_CRITICAL(&_sharedData.spinlock);
}

void ScannerApp::run() { step(); }
