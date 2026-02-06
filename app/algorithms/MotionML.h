/*
 * MotionML.h
 * 
 * TinyML-based motion classification
 * Uses Edge Impulse trained models to classify detected motion
 * 
 * Features:
 * - Integration with Edge Impulse runtime
 * - Configurable classification probability threshold
 * - Support for grayscale and RGB input
 * - Multiple object class detection
 * - Optional verbose logging of predictions
 */

#ifndef MOTION_ML_H
#define MOTION_ML_H

#include <Arduino.h>
#include "esp_camera.h"

// Forward declarations for Edge Impulse
#ifndef EI_CLASSIFIER_INPUT_WIDTH
#define EI_CLASSIFIER_INPUT_WIDTH 96
#define EI_CLASSIFIER_INPUT_HEIGHT 96
#endif

class MotionML {
public:
    // Configuration parameters
    struct Config {
        float minProbability = 0.8;      // Minimum confidence (0.0 - 1.0)
        bool useColor = false;           // false=grayscale, true=RGB
        bool verboseOutput = false;      // Enable detailed logging
        uint16_t inputWidth = 96;        // Model input width
        uint16_t inputHeight = 96;       // Model input height
    };

    // Classification result
    struct Result {
        bool classified;                 // True if probability > threshold
        float probability;               // Highest classification probability
        uint8_t classIndex;              // Index of classified object
        const char* className;           // Name of classified object
        uint32_t processingTime;         // Time taken for inference (ms)
        uint32_t dspTime;                // DSP processing time (ms)
        uint32_t inferenceTime;          // Neural network inference time (ms)
        uint32_t anomalyTime;            // Anomaly detection time (ms)
    };

    MotionML();
    ~MotionML();

    // Initialize classifier with configuration
    bool begin(const Config& config);
    
    // Update configuration
    void setConfig(const Config& config);
    void setProbabilityThreshold(float threshold);
    void setVerboseMode(bool enable);
    
    // Classify image buffer
    // Buffer should be prepared (resized, converted) before calling
    Result classify(const uint8_t* imageBuffer, size_t bufferSize);
    
    // Classify from camera frame
    Result classifyFrame(camera_fb_t* fb);
    
    // Get number of classes in model
    uint16_t getClassCount();
    
    // Get class name by index
    const char* getClassName(uint8_t index);
    
    // Check if TinyML is available/compiled
    static bool isAvailable();
    
    // Reset classifier state
    void reset();

private:
    Config _config;
    uint8_t _colorDepth;
    
    // Image processing buffer
    uint8_t* _inputBuffer;
    
    // Internal processing
    bool prepareInput(const uint8_t* imageBuffer, size_t bufferSize);
    void rescaleImage(const uint8_t* input, int inputWidth, int inputHeight,
                      uint8_t* output, int outputWidth, int outputHeight);
    bool jpg2rgb(const uint8_t* src, size_t src_len, uint8_t* out, uint8_t scale);
    void rgbToGray(uint8_t* buffer, int width, int height);
    
    // Edge Impulse integration
    Result runInference();
    static int getImageData(size_t offset, size_t length, float* out_ptr);
    
    // Static buffer for Edge Impulse callback
    static uint8_t* s_imageBuffer;
    static uint8_t s_colorDepth;
    
    // Helper functions
    void allocateBuffers();
    void freeBuffers();
};

#endif // MOTION_ML_H
