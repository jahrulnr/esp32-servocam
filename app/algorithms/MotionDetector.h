/*
 * MotionDetector.h
 * 
 * Native motion detection using background subtraction technique
 * Detects movement by comparing pixel changes between consecutive frames
 * 
 * Features:
 * - JPEG to bitmap conversion with configurable color depth
 * - Bilinear interpolation for image resizing to 96x96
 * - ROI (Region of Interest) based detection
 * - Adaptive threshold calculation
 * - Night/day detection
 * - Debug visualization support
 */

#ifndef MOTION_DETECTOR_H
#define MOTION_DETECTOR_H

#include <Arduino.h>
#include "esp_camera.h"

#define RESIZE_DIM 96
#define RESIZE_DIM_SQ (RESIZE_DIM * RESIZE_DIM)
#define RGB888_BYTES 3
#define GRAYSCALE_BYTES 1
#define INACTIVE_COLOR 96
#define JPEG_QUAL 80

class MotionDetector {
public:
    // Configuration parameters
    struct Config {
        int detectMotionFrames = 5;      // Min consecutive frames to confirm motion
        int detectNightFrames = 10;      // Frames to confirm night/day transition
        int detectNumBands = 10;         // Total horizontal bands
        int detectStartBand = 3;         // Top band for ROI
        int detectEndBand = 8;           // Bottom band for ROI (inclusive)
        int detectChangeThreshold = 15;  // Min pixel difference for change
        uint8_t nightSwitch = 20;        // Light level % threshold for night
        float motionVal = 8.0;           // Motion sensitivity (1-10)
        bool depthColor = false;         // false=grayscale, true=RGB
        bool dbgMotion = false;          // Enable debug visualization
    };

    // Detection result
    struct Result {
        bool motionDetected;
        uint8_t lightLevel;
        bool isNightTime;
        uint16_t changeCount;
        uint16_t threshold;
        uint32_t processingTime;
    };

    // Bounding box for motion region
    struct BoundingBox {
        bool valid;          // true if motion detected
        uint16_t x1, y1;     // Top-left corner
        uint16_t x2, y2;     // Bottom-right corner
        uint16_t centerX;    // Center X coordinate
        uint16_t centerY;    // Center Y coordinate
        uint16_t width;      // Image width (96)
        uint16_t height;     // Image height (96)
    };

    MotionDetector();
    ~MotionDetector();

    // Initialize detector with configuration
    void begin(const Config& config);
    
    // Update configuration parameters
    void setConfig(const Config& config);
    void setMotionSensitivity(float sensitivity);
    void setDebugMode(bool enable);
    void setColorDepth(bool useColor);
    
    // Main detection function
    Result checkMotion(camera_fb_t* fb, bool isCapturing = false);
    
    // Calculate light level only (no motion detection)
    uint8_t getLightLevel(camera_fb_t* fb);
    
    // Check if currently night time
    bool isNight();
    
    // Get debug JPEG for visualization
    bool getDebugJpeg(uint8_t** jpegBuf, size_t* jpegLen);
    
    // Get bounding box of detected motion region
    BoundingBox getMotionBoundingBox();
    
    // Reset detection state
    void reset();

private:
    Config _config;
    
    // State variables
    uint8_t _colorDepth;
    size_t _stride;
    uint8_t _lightLevel;
    bool _nightTime;
    uint32_t _motionCnt;
    int16_t _nightCnt;
    
    // Image buffers (allocated in PSRAM)
    uint8_t* _currBuff;
    uint8_t* _prevBuff;
    uint8_t* _changeMap;
    uint8_t* _rgbBuf;
    uint8_t* _motionJpeg;
    size_t _motionJpegLen;
    
    // Internal processing functions
    bool jpg2rgb(const uint8_t* src, size_t src_len, uint8_t* out, uint8_t scale);
    void rescaleImage(const uint8_t* input, int inputWidth, int inputHeight, 
                      uint8_t* output, int outputWidth, int outputHeight);
    void rgbToGray(uint8_t* buffer, int width, int height);
    
    // Detection logic
    int compareFrames(size_t resizeDimLen, uint16_t* startPixel, uint16_t* endPixel);
    void updateLightLevel(uint32_t lux);
    bool validateMotion(int changeCount, int threshold, bool motionStatus);
    bool generateDebugJpeg();
    
    // Helper functions
    void allocateBuffers();
    void freeBuffers();
    int calculateThreshold(uint16_t startPixel, uint16_t endPixel);
    BoundingBox extractBoundingBox();
};

#endif // MOTION_DETECTOR_H
