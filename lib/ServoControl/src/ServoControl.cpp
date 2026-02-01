#include "ServoControl.h"

ServoControl::ServoControl(int pin, int minAngle, int maxAngle, int defaultAngle, bool reverse)
  : _attached(false), _updaterCallback(NULL), _stepSize(2), _stepDelayMs(12), _lastUpdate(0) {
  _servo.pin = pin;

  defaultAngle = constrain(defaultAngle, minAngle, maxAngle);
  _servo.currentAngle = defaultAngle;
  _servo.targetAngle = defaultAngle;
  _servo.minAngle = minAngle;
  _servo.maxAngle = maxAngle;
  _servo.defaultAngle = defaultAngle;
  _servo.reverse = reverse;
  _servo.moving = false;
}

ServoControl::~ServoControl() {
  detach();
}

void ServoControl::attach() {
  if (!_attached) {
    _servo.servo.attach(_servo.pin, 500, 2500);
    int physicalAngle = _servo.reverse ? (_servo.minAngle + _servo.maxAngle - _servo.currentAngle) : _servo.currentAngle;
    _servo.servo.write(physicalAngle);
    _attached = true;
  }
  delay(50);
}

void ServoControl::detach() {
  delay(50);
  if (_attached) {
    _servo.servo.detach();
    _attached = false;
  }
}

void ServoControl::setAngle(int angle, bool smooth) {
  int logicalAngle = constrain(angle, _servo.minAngle, _servo.maxAngle);
  if (_servo.currentAngle == logicalAngle) return;
  _servo.targetAngle = logicalAngle;

  if (smooth) {
    _servo.moving = true;
    _lastUpdate = millis();
  } else {
    if (!_attached) attach();
    _servo.currentAngle = _servo.targetAngle;
    int physicalAngle = _servo.reverse ? (_servo.minAngle + _servo.maxAngle - _servo.currentAngle) : _servo.currentAngle;
    _servo.servo.write(physicalAngle);
    if (_updaterCallback) {
      _updaterCallback();
    }
  }
}

int ServoControl::getAngle() const {
  return _servo.currentAngle;
}

void ServoControl::reset() {
  if (_servo.moving || _servo.currentAngle == _servo.defaultAngle) return;
  setAngle(_servo.defaultAngle);
}

void ServoControl::setUpdaterCallback(void (*callback)(void)) {
  _updaterCallback = callback;
}

void ServoControl::update() {
  if (_servo.moving && millis() - _lastUpdate >= _stepDelayMs) {
    if (!_attached) attach();
    if (_servo.currentAngle < _servo.targetAngle) {
      _servo.currentAngle = min(_servo.currentAngle + _stepSize, _servo.targetAngle);
    } else if (_servo.currentAngle > _servo.targetAngle) {
      _servo.currentAngle = max(_servo.currentAngle - _stepSize, _servo.targetAngle);
    }
    int physicalAngle = _servo.reverse ? (_servo.minAngle + _servo.maxAngle - _servo.currentAngle) : _servo.currentAngle;
    _servo.servo.write(physicalAngle);
    _lastUpdate = millis();
    if (_servo.currentAngle == _servo.targetAngle) {
      _servo.moving = false;
      if (_updaterCallback) {
        _updaterCallback();
      }
    }
  }
}