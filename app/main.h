#pragma once

#include <sdkconfig.h>
#include <esp_camera.h>
#include <esp32-hal-log.h>
#include <soc/soc.h>
#include "CameraConfig.h"
#include <Config.h>
#include <Secret.h>
#include <WebSocketsServer.h>
#include <WiFiManager.h>
#include <FTPServer.h>
#include <gpt.h>
#include <soc/soc.h> // Include for RTC_CNTL_BROWN_OUT_REG
#include <soc/rtc_cntl_reg.h>
#include <ESP32PWM.h>
#include <ServoControl.h>