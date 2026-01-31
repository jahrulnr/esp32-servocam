#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <functional>
#include <FS.h>
#include <WiFi.h>
#include "HttpClient.h"
#include <Config.h>

// YOLO Object Detection Service
// Calls backend predict endpoint with image data

class ObjectDetectionService {
public:
    static const char* TAG() { return "ObjectDetectionService"; }

    // Detection result structure
    struct Detection {
        float x1, y1, x2, y2;  // Bounding box
        float confidence;
        int oid;  // Object ID
        String classification;
    };

    struct ObjectDetectionResult {
        std::vector<Detection> detections;
        int width;
        int height;
        bool success;
        String error;
    };

    ObjectDetectionService();
    ~ObjectDetectionService();

    // Initialize with backend URL
    bool init(const String& backendUrl = "http://192.168.18.250:8080/predict");

    // Feed image data for prediction (non-blocking)
    bool feed(const uint8_t* imageData, size_t size);

    // Check if result is ready
    bool isReady();

    // Get prediction result (consumes the result)
    ObjectDetectionResult getResult();

    // Get last result without consuming
    ObjectDetectionResult getLastResult() const;

    // Clear buffer
    void clear();

private:
    String _backendUrl;
    bool _initialized;
    bool _inProgress;
    ObjectDetectionResult _bufferedResult;
    TaskHandle_t _predictTaskHandle;

    // Internal HTTP client
    HttpClient _http;

    // Task function for async prediction
    static void predictTask(void* param);

    // Internal predict function
    void doPredict(uint8_t* imageData, size_t size);

    // Process API response
    void processResponse(const String& response);
};