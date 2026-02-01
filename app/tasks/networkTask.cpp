#include "tasks.h"
#include <SpiJsonDocument.h>
#include <web/router.h>
#include "../service/ObjectDetectionService.h"
#include "../handler/DetectionHandler.h"

extern ServoControl* xServo;
extern ServoControl* yServo;
WebServer server(WEBSERVER_PORT);
WebSocketsServer wsServer(WEBSOCKET_PORT);

void networkTask(void *param) {
	const char* TAG = "networkTask";

	TickType_t updateFrequency = 51;

	wifiManager.init(WIFI_HOSTNAME);
	wifiManager.addNetwork(WIFI_SSID, WIFI_PASS);
	wifiManager.begin();
	ftpServer.begin(FTP_USER, FTP_PASS);

	// Init GPT
  ai.init(GPT_API_KEY);
  ai.setSystemMessage(GPT_SYSTEM_MESSAGE);

	// Object detection service
	static ObjectDetectionService objectDetectionService;
	objectDetectionService.init(OBJECT_DETECTION_URL);

	// webserver
	setupWebRouter();

	ESP_LOGI(TAG, "Network task started");
	do {
		vTaskDelay(updateFrequency);
		
		wifiManager.handle();
		ftpServer.handleFTP();
  	server.handleClient();
		wsServer.loop();
		if (!wifiManager.isConnected()) continue;

		static unsigned long lastFrameBroadcastTime = 0;
		static unsigned long lastDetectionFeedTime = 0;

		// Decide if we need to get a frame:
		// - broadcast frames at ~20fps when there are WS clients
		// - feed detector every ~300ms regardless of WS clients
		bool needBroadcast = (millis() - lastFrameBroadcastTime > 50) && (wsClientsNum() > 0);
		bool needFeed = (millis() - lastDetectionFeedTime > 300);

		if (needBroadcast || needFeed) {
			auto frameBuffer = esp_camera_fb_get();
			if (!frameBuffer) {
				ESP_LOGW(TAG, "Failed to get camera frame");
			} else {
				if (needBroadcast) {
					ESP_LOGI(TAG, "Broadcasting frame (clients=%d, len=%u)", wsClientsNum(), (unsigned)frameBuffer->len);
					wsServer.broadcastBIN(frameBuffer->buf, frameBuffer->len);
					lastFrameBroadcastTime = millis();
				}

				if (needFeed) {
					// feed only if detector is not busy; feed() should return false if busy
					if (objectDetectionService.feed(frameBuffer->buf, frameBuffer->len)) {
						lastDetectionFeedTime = millis();
					} else {
						// if feed failed because busy, don't advance the timer so we'll retry next loop
					}
				}

				esp_camera_fb_return(frameBuffer);
			}
		}

		// Delegate detection result handling to handler
		handleDetections(objectDetectionService, wsServer, xServo, yServo);
	} while(1);

	WiFi.disconnect();
	wifiManager.stopHotspot();
	ftpServer.stop();
  server.close();
	vTaskDelete(NULL);
}