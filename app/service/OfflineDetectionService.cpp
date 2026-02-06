/*
 * OfflineDetectionService.cpp
 * 
 * Implementation of offline object detection using motion tracking
 */

#include "OfflineDetectionService.h"

OfflineDetectionService::OfflineDetectionService()
    : _resultReady(false)
    , _minMotionArea(100)  // Minimum 10x10 pixel area
{
    _bufferedResult.success = false;
}

OfflineDetectionService::~OfflineDetectionService() {
}

bool OfflineDetectionService::init(const MotionDetector::Config& config) {
    _detector.begin(config);
    ESP_LOGI(TAG(), "Offline detection service initialized (motion-based)");
    return true;
}

bool OfflineDetectionService::feed(camera_fb_t* fb) {
    if (!fb) {
        return false;
    }
    
    // Run motion detection
    MotionDetector::Result motionResult = _detector.checkMotion(fb, false);
    
    // Get bounding box if motion detected
    MotionDetector::BoundingBox bbox = _detector.getMotionBoundingBox();
    
    // Clear previous results
    _bufferedResult.detections.clear();
    _bufferedResult.success = false;
    
    // Check if we have valid motion with sufficient area
    if (bbox.valid) {
        uint16_t area = (bbox.x2 - bbox.x1) * (bbox.y2 - bbox.y1);
        
        if (area >= _minMotionArea) {
            // Convert to detection format
            Detection det = bboxToDetection(bbox);
            
            // Scale to camera resolution (assume SVGA 800x600 or similar)
            // You can adjust this based on actual camera config
            uint16_t camWidth = fb->width;
            uint16_t camHeight = fb->height;
            
            // If camera dimensions not available, use defaults
            if (camWidth == 0) camWidth = 800;
            if (camHeight == 0) camHeight = 600;
            
            scaleCoordinates(det, camWidth, camHeight);
            
            // Add to results
            _bufferedResult.detections.push_back(det);
            _bufferedResult.width = camWidth;
            _bufferedResult.height = camHeight;
            _bufferedResult.success = true;
            _bufferedResult.error = "";
            _resultReady = true;
            
            ESP_LOGD(TAG(), "Motion detected: bbox[%d,%d,%d,%d] area=%d", 
                     det.x1, det.y1, det.x2, det.y2, area);
        } else {
            ESP_LOGD(TAG(), "Motion area too small: %d < %d", area, _minMotionArea);
            _resultReady = false;
        }
    } else {
        _resultReady = false;
    }
    
    return true;
}

bool OfflineDetectionService::isReady() {
    return _resultReady && _bufferedResult.success;
}

OfflineDetectionService::ObjectDetectionResult OfflineDetectionService::getResult() {
    ObjectDetectionResult result = _bufferedResult;
    _bufferedResult.success = false;  // Consume
    _bufferedResult.detections.clear();
    _resultReady = false;
    return result;
}

OfflineDetectionService::ObjectDetectionResult OfflineDetectionService::getLastResult() const {
    return _bufferedResult;
}

void OfflineDetectionService::clear() {
    _bufferedResult.success = false;
    _bufferedResult.detections.clear();
    _bufferedResult.error = "";
    _resultReady = false;
}

void OfflineDetectionService::setMotionSensitivity(float sensitivity) {
    _detector.setMotionSensitivity(sensitivity);
}

void OfflineDetectionService::setMinMotionArea(uint16_t minArea) {
    _minMotionArea = minArea;
}

OfflineDetectionService::Detection OfflineDetectionService::bboxToDetection(
    const MotionDetector::BoundingBox& bbox) {
    
    Detection det;
    
    // Copy bounding box coordinates (still in 96x96 space)
    det.x1 = bbox.x1;
    det.y1 = bbox.y1;
    det.x2 = bbox.x2;
    det.y2 = bbox.y2;
    
    // Motion detection doesn't classify, so use generic label
    det.classification = "motion";
    det.oid = 0;
    
    // Calculate confidence based on motion intensity
    // Larger bounding boxes = higher confidence
    uint16_t area = (bbox.x2 - bbox.x1) * (bbox.y2 - bbox.y1);
    float maxArea = (float)(RESIZE_DIM * RESIZE_DIM);
    det.confidence = min(0.95f, 0.5f + (area / maxArea) * 0.5f);  // 0.5 - 0.95 range
    
    return det;
}

void OfflineDetectionService::scaleCoordinates(Detection& det, 
                                                 uint16_t targetWidth, 
                                                 uint16_t targetHeight) {
    // Scale from 96x96 to target resolution
    float scaleX = (float)targetWidth / (float)RESIZE_DIM;
    float scaleY = (float)targetHeight / (float)RESIZE_DIM;
    
    det.x1 = (int)(det.x1 * scaleX);
    det.y1 = (int)(det.y1 * scaleY);
    det.x2 = (int)(det.x2 * scaleX);
    det.y2 = (int)(det.y2 * scaleY);
    
    // Clamp to valid range
    det.x1 = constrain(det.x1, 0, targetWidth - 1);
    det.x2 = constrain(det.x2, 0, targetWidth - 1);
    det.y1 = constrain(det.y1, 0, targetHeight - 1);
    det.y2 = constrain(det.y2, 0, targetHeight - 1);
}
