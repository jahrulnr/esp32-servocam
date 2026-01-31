#include "tasks.h"
#include <esp_camera.h>

void monitorerTask(void* param) {
	const char* TAG = "monitorerTask";

	TickType_t lastWakeTime = xTaskGetTickCount();
	TickType_t updateFrequency = pdMS_TO_TICKS(103);
	unsigned long monitorTimer = millis();
	unsigned long monitorDelay = 10000;
	unsigned long lastUpdate = 0;
	int lastHour = 0;

	ESP_LOGI(TAG, "Monitorer task started");
	vTaskDelay(pdMS_TO_TICKS(monitorDelay));
	do {
		vTaskDelay(updateFrequency);

		// task switcher
		vPortYieldOtherCore(0);
		delay(1);
		vPortYieldOtherCore(1);

		if (!STORAGE.exists("/cache/sample.jpg")) {
			camera_fb_t* fb = esp_camera_fb_get();
			if (fb){
				File file = STORAGE.open("/cache/sample.jpg", FILE_WRITE);
				if (file){
					file.write(fb->buf, fb->len);
					file.close();
				} else {
					ESP_LOGE("cam", "failed to write buffer");
				}
				esp_camera_fb_return(fb);
			} else {
				ESP_LOGE("cam", "failed to get buffer");
			}
		}
	}
	while(1);
}