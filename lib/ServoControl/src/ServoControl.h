#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <vector>

struct Servo_t {
	Servo servo;
	int pin;
	int currentAngle;
	int targetAngle;
	int defaultAngle;
	int minAngle;
	int maxAngle;
	bool reverse;
	bool moving;
};

class ServoControl {
private:
	Servo_t _servo;
	volatile bool _attached;
	TimerHandle_t _moveTimer;
	void (*_updaterCallback)(void);
	int _stepSize;
	int _stepDelayMs;
	unsigned long _lastUpdate;

public:
	ServoControl(int pin, int minAngle = 0, int maxAngle = 180, int defaultAngle = 90, bool reverse = false);
	~ServoControl();
	void attach();
	void detach();
	void setAngle(int angle, bool smooth = true);
	int getAngle() const;
	void reset();
	void setUpdaterCallback(void (*callback)(void));
	void update(); // Update servo movement based on millis
	bool status() const { return _servo.moving; } 
	unsigned long lastUpdate() const { return _lastUpdate; }
};

#endif // SERVO_CONTROL_H