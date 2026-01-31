#include "tasks.h"

extern ServoControl* xServo;
extern ServoControl* yServo;

void handlerTask(void* param) {
	TickType_t updateFrequency = 1;
	
	do{
		vTaskDelay(updateFrequency);
		xServo->update();
		yServo->update();
	}while(1);
}