#include <Arduino.h>
#include <main.h>
#include "tasks/tasks.h"

FTPServer ftpServer(STORAGE);
ServoControl* xServo;
ServoControl* yServo;

int I2Cdevices = 0;
void init() {
  heap_caps_malloc_extmem_enable(0);
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  STORAGE.begin(true);
}

void setup() {
  ai.init(GPT_API_KEY);
  ai.setSystemMessage(GPT_SYSTEM_MESSAGE);

	#ifdef ESP32
		WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
	#endif

  camera_config_t config;
	config.ledc_channel = LEDC_CHANNEL_0;
	config.ledc_timer = LEDC_TIMER_0;
	config.pin_d0 = Y2_GPIO_NUM;
	config.pin_d1 = Y3_GPIO_NUM;
	config.pin_d2 = Y4_GPIO_NUM;
	config.pin_d3 = Y5_GPIO_NUM;
	config.pin_d4 = Y6_GPIO_NUM;
	config.pin_d5 = Y7_GPIO_NUM;
	config.pin_d6 = Y8_GPIO_NUM;
	config.pin_d7 = Y9_GPIO_NUM;
	config.pin_xclk = XCLK_GPIO_NUM;
	config.pin_pclk = PCLK_GPIO_NUM;
	config.pin_vsync = VSYNC_GPIO_NUM;
	config.pin_href = HREF_GPIO_NUM;
	config.pin_sccb_sda = SIOD_GPIO_NUM;
	config.pin_sccb_scl = SIOC_GPIO_NUM;
	config.pin_pwdn = PWDN_GPIO_NUM;
	config.pin_reset = RESET_GPIO_NUM;
	config.xclk_freq_hz = 20 *1000*1000;
	config.pixel_format = CAMERA_PIXEL_FORMAT;
	config.jpeg_quality = CAMERA_QUALITY;  // 0-63, lower is better quality
	config.frame_size = CAMERA_FRAME_SIZE;
  config.grab_mode = psramFound() 
		? CAMERA_GRAB_LATEST 
		: CAMERA_GRAB_WHEN_EMPTY;
  config.fb_count = psramFound() ? 2 : 1;
  config.fb_location = psramFound() 
    ? CAMERA_FB_IN_PSRAM
    : CAMERA_FB_IN_DRAM;

	esp_err_t err = esp_camera_init(&config);
	if (err != ESP_OK) {
		ESP_LOGE("Camera", "init failed with error 0x%x (%s)", err, esp_err_to_name(err));
	}
  delay(1);

	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
  xServo = new ServoControl(X_SERVO_PIN, 60, 120, DEFAULT_X_ANGLE);
  yServo = new ServoControl(Y_SERVO_PIN, 0, 60, DEFAULT_Y_ANGLE);
	delay(1);
  xServo->setAngle(DEFAULT_X_ANGLE+1); // trigger update
  delay(1);
  yServo->setAngle(DEFAULT_Y_ANGLE+1); // trigger update

	runTasks();
}

void loop() {
  vTaskDelete(NULL);
}
