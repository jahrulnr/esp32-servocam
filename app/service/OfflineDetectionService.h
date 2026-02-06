/*
 * OfflineDetectionService.h
 * 
 * Offline object detection using MotionDetector
 * Provides ObjectDetectionService-compatible interface for servo tracking
 */

#ifndef OFFLINE_DETECTION_SERVICE_H
#define OFFLINE_DETECTION_SERVICE_H

#include <Arduino.h>
#include <vector>
#include "ObjectDetectionService.h"
#include "algorithms/MotionDetector.h"

class OfflineDetectionService {
public:
    using Detection = ObjectDetectionService::Detection;
    using ObjectDetectionResult = ObjectDetectionService::ObjectDetectionResult;

    OfflineDetectionService();
    ~OfflineDetectionService();

    // Initialize with motion detector config
    bool init(const MotionDetector::Config& config);
    
    // Feed camera frame for detection (compatible with ObjectDetectionService)
    bool feed(camera_fb_t* fb);
    
    // Check if result is ready
    bool isReady();
    
    // Get detection result (consumes result)
    ObjectDetectionResult getResult();
    
    // Get last result without consuming
    ObjectDetectionResult getLastResult() const;
    
    // Clear buffered result
    void clear();
    
    // Configuration
    void setMotionSensitivity(float sensitivity);
    void setMinMotionArea(uint16_t minArea);  // Minimum pixel area to report
    
    // Get underlying motion detector for advanced control
    MotionDetector& getMotionDetector() { return _detector; }

private:
    MotionDetector _detector;
    ObjectDetectionResult _bufferedResult;
    bool _resultReady;
    uint16_t _minMotionArea;  // Minimum bounding box area to report
    
    // Convert MotionDetector bbox to Detection
    Detection bboxToDetection(const MotionDetector::BoundingBox& bbox);
    
    // Scale coordinates from 96x96 to camera resolution
    void scaleCoordinates(Detection& det, uint16_t targetWidth, uint16_t targetHeight);
    
    static const char* TAG() { return "OfflineDetection"; }
};

#endif // OFFLINE_DETECTION_SERVICE_H
