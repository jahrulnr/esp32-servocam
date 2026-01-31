#pragma once

#include <Arduino.h>
#include "../service/ObjectDetectionService.h"
#include <WebSocketsServer.h>
#include <SpiJsonDocument.h>
#include <ServoControl.h>

// Handle ready detection results: broadcast JSON and command servos
void handleDetections(ObjectDetectionService &objectDetectionService, WebSocketsServer &wsServer, ServoControl* xServo, ServoControl* yServo);
