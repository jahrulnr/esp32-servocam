#include "tasks.h"
#include <SpiJsonDocument.h>
#include <web/router.h>

WebServer server(WEBSERVER_PORT);
WebSocketsServer wsServer(WEBSOCKET_PORT);

void networkTask(void *param) {
	const char* TAG = "networkTask";

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

		static unsigned long sendImage = 0;
		if (millis() - sendImage > 50 && wsClientsNum() > 0){
			auto fb = esp_camera_fb_get();
			wsServer.broadcastBIN(fb->buf, fb->len);
			esp_camera_fb_return(fb);
			sendImage = millis();
		}
	} while(1);

	WiFi.disconnect();
	wifiManager.stopHotspot();
	ftpServer.stop();
  server.close();
	vTaskDelete(NULL);
}