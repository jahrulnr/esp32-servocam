#include "tasks.h"
#include <SpiJsonDocument.h>
#include <web/router.h>
#include "../service/ObjectDetectionService.h"

WebServer server(WEBSERVER_PORT);
WebSocketsServer wsServer(WEBSOCKET_PORT);

void networkTask(void *param) {
	const char* TAG = "networkTask";

	// Object detection service
	static ObjectDetectionService objectDetectionService;
	objectDetectionService.init();

	TickType_t updateFrequency = 51;
	unsigned long monitorCheck = millis();
	unsigned long timeCheck = 0;
	unsigned long weatherCheck = 0;
	unsigned long gptCheck = 0;
	const char* lastEvent;
	
	wifiManager.init(WIFI_HOSTNAME);
	wifiManager.addNetwork(WIFI_SSID, WIFI_PASS);
	wifiManager.begin();
	ftpServer.begin(FTP_USER, FTP_PASS);

	// Init GPT
  ai.init(GPT_API_KEY);
  ai.setSystemMessage(GPT_SYSTEM_MESSAGE);

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
		if (millis() - lastFrameBroadcastTime > 50 && wsClientsNum() > 0){
			auto frameBuffer = esp_camera_fb_get();
			wsServer.broadcastBIN(frameBuffer->buf, frameBuffer->len);
			// Try feeding object detection at a lower rate (e.g., every 1000ms)
			if (millis() - lastDetectionFeedTime > 1000) {
				// feed only if detector is not busy
				objectDetectionService.feed(frameBuffer->buf, frameBuffer->len);
				lastDetectionFeedTime = millis();
			}
			esp_camera_fb_return(frameBuffer);
			lastFrameBroadcastTime = millis();
		}

		// If detection result ready, get and broadcast as JSON
		if (objectDetectionService.isReady()) {
			auto detectionResult = objectDetectionService.getResult();
			SpiJsonDocument detectionJson;
			detectionJson["type"] = "detections";
			detectionJson["width"] = detectionResult.width;
			detectionJson["height"] = detectionResult.height;
			JsonArray detectionsArray = detectionJson.createNestedArray("detections");
			for (auto &det : detectionResult.detections) {
				SpiJsonDocument obj;
				JsonArray box = obj.createNestedArray("box");
				box.add(det.x1);
				box.add(det.y1);
				box.add(det.x2);
				box.add(det.y2);
				obj["confidence"] = det.confidence;
				obj["oid"] = det.oid;
				obj["classification"] = det.classification;
				detectionsArray.add(obj);
			}
			String outTxt;
			serializeJson(detectionJson, outTxt);
			wsServer.broadcastTXT(outTxt);
		}
	} while(1);

	WiFi.disconnect();
	wifiManager.stopHotspot();
	ftpServer.stop();
  server.close();
	vTaskDelete(NULL);
}