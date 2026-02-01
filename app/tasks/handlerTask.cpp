#include "tasks.h"

extern ServoControl* xServo;
extern ServoControl* yServo;

void handlerTask(void* param) {
	TickType_t updateFrequency = 1;
	const unsigned long INACTIVITY_MS = 10000UL; // 10 seconds

	do{
		vTaskDelay(updateFrequency);
		xServo->update();
		yServo->update();

		unsigned long now = millis();
		// If a servo hasn't updated in INACTIVITY_MS and is not moving, reset it to default
		if (xServo != nullptr) {
			unsigned long last = xServo->lastUpdate();
			if (last != 0 && (now - last) > INACTIVITY_MS && !xServo->status()) {
				xServo->reset();
			}
		}
		if (yServo != nullptr) {
			unsigned long last = yServo->lastUpdate();
			if (last != 0 && (now - last) > INACTIVITY_MS && !yServo->status()) {
				yServo->reset();
			}
		}
	} while(1);
}