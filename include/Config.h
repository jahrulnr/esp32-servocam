#pragma once
#include <Secret.h>
#include <LittleFS.h>
#include <HW.h>

#define STORAGE LittleFS

// Web server configuration
#define WEBSERVER_PORT 80
#define WEBSOCKET_PORT 81

// Enable/disable features for testing
// Set to 0 to disable object detection handling (useful for testing awareness search)
#define ENABLE_OBJECT_DETECTION 1
#define OBJECT_DETECTION_URL "http://192.168.18.250:8080/predict"

// Awareness context: when no object detected, perform grid-search sweep
#define ENABLE_AWARENESS_CONTEXT 1

// Awareness grid settings
#define AWARENESS_GRID_ROWS 3
#define AWARENESS_GRID_COLS 3
#define AWARENESS_GRID_DWELL_MS 1200
