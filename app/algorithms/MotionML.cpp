/*
 * MotionML.cpp
 * 
 * Implementation of TinyML motion classification
 */

#include "MotionML.h"

// Check if TinyML is enabled
#ifdef INCLUDE_TINYML
#include TINY_ML_LIB

// Static members for Edge Impulse callback
uint8_t* MotionML::s_imageBuffer = nullptr;
uint8_t MotionML::s_colorDepth = 1;

MotionML::MotionML()
    : _colorDepth(1)
    , _inputBuffer(nullptr)
{
}

MotionML::~MotionML() {
    freeBuffers();
}

bool MotionML::begin(const Config& config) {
    _config = config;
    _colorDepth = _config.useColor ? 3 : 1; // RGB or Grayscale
    s_colorDepth = _colorDepth;
    
    allocateBuffers();
    
    return true;
}

void MotionML::setConfig(const Config& config) {
    _config = config;
    _colorDepth = _config.useColor ? 3 : 1;
    s_colorDepth = _colorDepth;
}

void MotionML::setProbabilityThreshold(float threshold) {
    _config.minProbability = constrain(threshold, 0.0f, 1.0f);
}

void MotionML::setVerboseMode(bool enable) {
    _config.verboseOutput = enable;
}

MotionML::Result MotionML::classify(const uint8_t* imageBuffer, size_t bufferSize) {
    Result result = {false, 0.0, 0, "", 0, 0, 0, 0};
    uint32_t startTime = millis();
    
    if (!imageBuffer || bufferSize == 0) {
        return result;
    }
    
    // Prepare input buffer
    if (!prepareInput(imageBuffer, bufferSize)) {
        return result;
    }
    
    // Run inference
    result = runInference();
    result.processingTime = millis() - startTime;
    
    return result;
}

MotionML::Result MotionML::classifyFrame(camera_fb_t* fb) {
    Result result = {false, 0.0, 0, "", 0, 0, 0, 0};
    
    if (!fb) {
        return result;
    }
    
    // Allocate temporary buffer for JPEG decoding
    uint8_t* rgbBuf = (uint8_t*)ps_malloc(800 * 600 * 3 / 8);
    if (!rgbBuf) {
        return result;
    }
    
    // JPEG to RGB
    if (jpg2rgb(fb->buf, fb->len, rgbBuf, 2)) {
        // Resize to model input size
        rescaleImage(rgbBuf, 100, 75, _inputBuffer, _config.inputWidth, _config.inputHeight);
        
        // Convert to grayscale if needed
        if (_colorDepth == 1) {
            rgbToGray(_inputBuffer, _config.inputWidth, _config.inputHeight);
        }
        
        // Run classification
        result = runInference();
    }
    
    free(rgbBuf);
    return result;
}

uint16_t MotionML::getClassCount() {
#ifdef EI_CLASSIFIER_LABEL_COUNT
    return EI_CLASSIFIER_LABEL_COUNT;
#else
    return 0;
#endif
}

const char* MotionML::getClassName(uint8_t index) {
#ifdef EI_CLASSIFIER_LABEL_COUNT
    if (index < EI_CLASSIFIER_LABEL_COUNT) {
        return ei_classifier_inferencing_categories[index];
    }
#endif
    return "unknown";
}

bool MotionML::isAvailable() {
#ifdef INCLUDE_TINYML
    return true;
#else
    return false;
#endif
}

void MotionML::reset() {
    if (_inputBuffer) {
        memset(_inputBuffer, 0, _config.inputWidth * _config.inputHeight * _colorDepth);
    }
}

// Private methods

void MotionML::allocateBuffers() {
    size_t bufferSize = _config.inputWidth * _config.inputHeight * 3; // Max size (RGB)
    if (!_inputBuffer) {
        _inputBuffer = (uint8_t*)ps_malloc(bufferSize);
    }
}

void MotionML::freeBuffers() {
    if (_inputBuffer) {
        free(_inputBuffer);
        _inputBuffer = nullptr;
    }
}

bool MotionML::prepareInput(const uint8_t* imageBuffer, size_t bufferSize) {
    if (!_inputBuffer) {
        return false;
    }
    
    // Check if resize is needed
    size_t expectedSize = _config.inputWidth * _config.inputHeight * _colorDepth;
    
    if (bufferSize == expectedSize) {
        // Direct copy
        memcpy(_inputBuffer, imageBuffer, bufferSize);
    } else {
        // Need to resize (assuming square input for simplicity)
        int inputDim = sqrt(bufferSize / _colorDepth);
        rescaleImage(imageBuffer, inputDim, inputDim, 
                    _inputBuffer, _config.inputWidth, _config.inputHeight);
    }
    
    s_imageBuffer = _inputBuffer;
    return true;
}

MotionML::Result MotionML::runInference() {
    Result result = {false, 0.0, 0, "", 0, 0, 0, 0};
    
#ifdef INCLUDE_TINYML
    // Setup signal for Edge Impulse
    signal_t features_signal;
    features_signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    features_signal.get_data = &MotionML::getImageData;
    
    // Run classifier
    ei_impulse_result_t ei_result = {0};
    EI_IMPULSE_ERROR res = run_classifier(&features_signal, &ei_result, false);
    
    if (res == EI_IMPULSE_OK) {
        // Get timing
        result.dspTime = ei_result.timing.dsp;
        result.inferenceTime = ei_result.timing.classification;
        result.anomalyTime = ei_result.timing.anomaly;
        
        // Find highest probability
        float maxProb = 0.0;
        uint8_t maxIndex = 0;
        
        for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            if (ei_result.classification[i].value > maxProb) {
                maxProb = ei_result.classification[i].value;
                maxIndex = i;
            }
        }
        
        result.probability = maxProb;
        result.classIndex = maxIndex;
        result.className = ei_classifier_inferencing_categories[maxIndex];
        result.classified = (maxProb > _config.minProbability);
        
        // Verbose output
        if (_config.verboseOutput) {
            Serial.printf("ML Classification: %s (%.2f)\n", result.className, result.probability);
            Serial.printf("Timing: DSP %dms, Inference %dms, Anomaly %dms\n",
                         result.dspTime, result.inferenceTime, result.anomalyTime);
            
            Serial.print("All predictions: ");
            for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                Serial.printf("%s: %.2f ", 
                             ei_classifier_inferencing_categories[i],
                             ei_result.classification[i].value);
            }
            Serial.println();
        }
    } else {
        Serial.printf("Failed to run classifier: %d\n", res);
    }
#endif
    
    return result;
}

int MotionML::getImageData(size_t offset, size_t length, float* out_ptr) {
    if (!s_imageBuffer) {
        return -1;
    }
    
    size_t pixelPtr = offset * s_colorDepth;
    size_t out_ptr_idx = 0;
    
    while (out_ptr_idx < length) {
        if (s_colorDepth == 3) {
            // RGB
            out_ptr[out_ptr_idx++] = (float)((s_imageBuffer[pixelPtr] << 16) + 
                                             (s_imageBuffer[pixelPtr + 1] << 8) + 
                                             s_imageBuffer[pixelPtr + 2]);
        } else {
            // Grayscale
            out_ptr[out_ptr_idx++] = (float)((s_imageBuffer[pixelPtr] << 16) + 
                                             (s_imageBuffer[pixelPtr] << 8) + 
                                             s_imageBuffer[pixelPtr]);
        }
        pixelPtr += s_colorDepth;
    }
    
    return 0;
}

void MotionML::rescaleImage(const uint8_t* input, int inputWidth, int inputHeight,
                             uint8_t* output, int outputWidth, int outputHeight) {
    float xRatio = (float)inputWidth / (float)outputWidth;
    float yRatio = (float)inputHeight / (float)outputHeight;

    for (int i = 0; i < outputHeight; ++i) {
        for (int j = 0; j < outputWidth; ++j) {
            int xL = (int)floor(xRatio * j);
            int yL = (int)floor(yRatio * i);
            int xH = (int)ceil(xRatio * j);
            int yH = (int)ceil(yRatio * i);
            float xWeight = xRatio * j - xL;
            float yWeight = yRatio * i - yL;
            
            for (int channel = 0; channel < _colorDepth; ++channel) {
                uint8_t a = input[(yL * inputWidth + xL) * _colorDepth + channel];
                uint8_t b = input[(yL * inputWidth + xH) * _colorDepth + channel];
                uint8_t c = input[(yH * inputWidth + xL) * _colorDepth + channel];
                uint8_t d = input[(yH * inputWidth + xH) * _colorDepth + channel];

                float pixel = a * (1 - xWeight) * (1 - yWeight) + b * xWeight * (1 - yWeight)
                            + c * yWeight * (1 - xWeight) + d * xWeight * yWeight;
                output[(i * outputWidth + j) * _colorDepth + channel] = (uint8_t)pixel;
            }
        }
    }
}

void MotionML::rgbToGray(uint8_t* buffer, int width, int height) {
    for (int i = 0; i < width * height; ++i) {
        int index = i * 3;
        buffer[i] = (uint8_t)(((77 * buffer[index]) + (150 * buffer[index + 1]) + (29 * buffer[index + 2])) >> 8);
    }
}

bool MotionML::jpg2rgb(const uint8_t* src, size_t src_len, uint8_t* out, uint8_t scale) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0)
    static uint8_t work[3100];
    
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = (uint8_t *)src,
        .indata_size = src_len,
        .outbuf = out,
        .outbuf_size = UINT32_MAX,
        .out_format = JPEG_IMAGE_FORMAT_RGB888,
        .out_scale = (esp_jpeg_image_scale_t)scale,
        .flags = {.swap_color_bytes = 0},
        .advanced = {
            .working_buffer = work,
            .working_buffer_size = sizeof(work)
        }
    };
    
    esp_jpeg_image_output_t output_img = {};
    esp_err_t res = esp_jpeg_decode(&jpeg_cfg, &output_img);
    return (res == ESP_OK);
#else
    return jpg2rgb888(src, src_len, out, (jpg_scale_t)scale);
#endif
}

#else // INCLUDE_TINYML not defined

// Stub implementations when TinyML is not available
uint8_t* MotionML::s_imageBuffer = nullptr;
uint8_t MotionML::s_colorDepth = 1;

MotionML::MotionML() : _colorDepth(1), _inputBuffer(nullptr) {}
MotionML::~MotionML() {}
bool MotionML::begin(const Config& config) { return false; }
void MotionML::setConfig(const Config& config) {}
void MotionML::setProbabilityThreshold(float threshold) {}
void MotionML::setVerboseMode(bool enable) {}
MotionML::Result MotionML::classify(const uint8_t* imageBuffer, size_t bufferSize) { 
    return {false, 0.0, 0, "", 0, 0, 0, 0}; 
}
MotionML::Result MotionML::classifyFrame(camera_fb_t* fb) { 
    return {false, 0.0, 0, "", 0, 0, 0, 0}; 
}
uint16_t MotionML::getClassCount() { return 0; }
const char* MotionML::getClassName(uint8_t index) { return "unavailable"; }
bool MotionML::isAvailable() { return false; }
void MotionML::reset() {}
void MotionML::allocateBuffers() {}
void MotionML::freeBuffers() {}
bool MotionML::prepareInput(const uint8_t* imageBuffer, size_t bufferSize) { return false; }
MotionML::Result MotionML::runInference() { return {false, 0.0, 0, "", 0, 0, 0, 0}; }
int MotionML::getImageData(size_t offset, size_t length, float* out_ptr) { return -1; }
void MotionML::rescaleImage(const uint8_t* input, int inputWidth, int inputHeight,
                            uint8_t* output, int outputWidth, int outputHeight) {}
void MotionML::rgbToGray(uint8_t* buffer, int width, int height) {}
bool MotionML::jpg2rgb(const uint8_t* src, size_t src_len, uint8_t* out, uint8_t scale) { return false; }

#endif // INCLUDE_TINYML
