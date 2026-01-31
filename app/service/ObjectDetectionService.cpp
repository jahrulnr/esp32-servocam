#include "ObjectDetectionService.h"
#include <SpiJsonDocument.h>

ObjectDetectionService::ObjectDetectionService()
    : _initialized(false), _inProgress(false), _predictTaskHandle(NULL) {
    _bufferedResult.success = false;
}

ObjectDetectionService::~ObjectDetectionService() {
    if (_predictTaskHandle != NULL) {
        vTaskDelete(_predictTaskHandle);
    }
}

bool ObjectDetectionService::init(const String& backendUrl) {
    _backendUrl = backendUrl;
    _initialized = true;
    ESP_LOGI(TAG(), "Predict service initialized with URL: %s", _backendUrl.c_str());
    return true;
}

bool ObjectDetectionService::feed(const uint8_t* imageData, size_t size) {
    if (!_initialized || _inProgress) {
        return false;
    }

    _inProgress = true;

    // Copy data to heap for task
    uint8_t* dataCopy = (uint8_t*)malloc(size);
    if (!dataCopy) {
        ESP_LOGE(TAG(), "Failed to allocate memory for image data");
        _inProgress = false;
        return false;
    }
    memcpy(dataCopy, imageData, size);

    // Create task parameters
    struct TaskParams {
        ObjectDetectionService* service;
        uint8_t* data;
        size_t size;
    };

    TaskParams* params = new TaskParams{this, dataCopy, size};

    xTaskCreate([](void* param){
        TaskParams* p = (TaskParams*)param;
        p->service->doPredict(p->data, p->size);
        free(p->data);
        delete p;
        vTaskDelete(NULL);
    }, "predictTask", 16*1024, params, 1, NULL);

    return true;
}

void ObjectDetectionService::predictTask(void* param) {
    // Not used
}

void ObjectDetectionService::doPredict(uint8_t* imageData, size_t size) {
    const String filename = "image.jpg";
    ESP_LOGD(TAG(), "Starting prediction task");

    _http.begin(_backendUrl);
    _http.setReuse(false);
    _http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    _http.setTimeout(30000); // 30 second timeout

    // Set headers for multipart
    String boundary = "----ESP32Boundary" + String(random(100000));
    _http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);

    // Build multipart body
    String bodyStart = "--" + boundary + "\r\n";
    bodyStart += "Content-Disposition: form-data; name=\"file\"; filename=\"" + filename + "\"\r\n";
    bodyStart += "Content-Type: image/jpeg\r\n\r\n";

    String bodyEnd = "\r\n--" + boundary + "--\r\n";

    size_t totalSize = bodyStart.length() + size + bodyEnd.length();

    // Allocate buffer for full body
    char* bodyBuffer = (char*)malloc(totalSize + 1);
    if (!bodyBuffer) {
        ESP_LOGE(TAG(), "Failed to allocate body buffer");
        _bufferedResult.success = false;
        _bufferedResult.error = "Memory allocation failed";
        _http.end();
        _inProgress = false;
        return;
    }

    // Copy parts
    memcpy(bodyBuffer, bodyStart.c_str(), bodyStart.length());
    memcpy(bodyBuffer + bodyStart.length(), imageData, size);
    memcpy(bodyBuffer + bodyStart.length() + size, bodyEnd.c_str(), bodyEnd.length());
    bodyBuffer[totalSize] = '\0';

    // Send POST request
    int httpCode = _http.POST((uint8_t*)bodyBuffer, totalSize);

    free(bodyBuffer);

    if (httpCode == HTTP_CODE_OK) {
        String response = _http.getString();
        ESP_LOGD(TAG(), "Prediction response received, length: %d", response.length());
        processResponse(response);
    } else {
        ESP_LOGE(TAG(), "HTTP error: %d", httpCode);
        _bufferedResult.success = false;
        _bufferedResult.error = "HTTP error: " + String(httpCode);
    }

    _http.end();
    _inProgress = false;
}

void ObjectDetectionService::processResponse(const String& response) {
    _bufferedResult.detections.clear();
    _bufferedResult.success = false;

    SpiJsonDocument doc;
    DeserializationError error = deserializeJson(doc, response);

    if (error) {
        ESP_LOGE(TAG(), "JSON parse error: %s", error.c_str());
        _bufferedResult.error = "JSON parse error: " + String(error.c_str());
        return;
    }

    if (doc.containsKey("detections")) {
        JsonArray detections = doc["detections"];
        for (JsonObject det : detections) {
            Detection d;
            JsonArray box = det["box"];
            if (box.size() == 4) {
                d.x1 = box[0];
                d.y1 = box[1];
                d.x2 = box[2];
                d.y2 = box[3];
            }
            d.confidence = det["confidence"];
            d.oid = det["oid"];
            d.classification = det["classification"].as<String>();
            _bufferedResult.detections.push_back(d);
        }
        _bufferedResult.width = doc["width"];
        _bufferedResult.height = doc["height"];
        _bufferedResult.success = true;
        _bufferedResult.error = "";
        ESP_LOGI(TAG(), "Parsed %d detections", _bufferedResult.detections.size());
    } else {
        _bufferedResult.error = "No detections in response";
    }
}

bool ObjectDetectionService::isReady() {
    return !_inProgress && _bufferedResult.success;
}

ObjectDetectionService::ObjectDetectionResult ObjectDetectionService::getResult() {
    ObjectDetectionResult result = _bufferedResult;
    _bufferedResult.success = false; // Consume
    _bufferedResult.detections.clear();
    return result;
}

ObjectDetectionService::ObjectDetectionResult ObjectDetectionService::getLastResult() const {
    return _bufferedResult;
}

void ObjectDetectionService::clear() {
    _bufferedResult.success = false;
    _bufferedResult.detections.clear();
    _bufferedResult.error = "";
}