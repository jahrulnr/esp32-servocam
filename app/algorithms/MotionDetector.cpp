/*
 * MotionDetector.cpp
 * 
 * Implementation of native motion detection using background subtraction
 */

#include "MotionDetector.h"
#include <esp_camera.h>
#include "img_converters.h"

MotionDetector::MotionDetector() 
    : _colorDepth(GRAYSCALE_BYTES)
    , _stride(RGB888_BYTES)
    , _lightLevel(50)
    , _nightTime(false)
    , _motionCnt(0)
    , _nightCnt(0)
    , _currBuff(nullptr)
    , _prevBuff(nullptr)
    , _changeMap(nullptr)
    , _rgbBuf(nullptr)
    , _motionJpeg(nullptr)
    , _motionJpegLen(0)
{
}

MotionDetector::~MotionDetector() {
    freeBuffers();
}

void MotionDetector::begin(const Config& config) {
    _config = config;
    _colorDepth = _config.depthColor ? RGB888_BYTES : GRAYSCALE_BYTES;
    _stride = (_colorDepth == RGB888_BYTES) ? GRAYSCALE_BYTES : RGB888_BYTES;
    allocateBuffers();
    reset();
}

void MotionDetector::setConfig(const Config& config) {
    _config = config;
    _colorDepth = _config.depthColor ? RGB888_BYTES : GRAYSCALE_BYTES;
    _stride = (_colorDepth == RGB888_BYTES) ? GRAYSCALE_BYTES : RGB888_BYTES;
}

void MotionDetector::setMotionSensitivity(float sensitivity) {
    _config.motionVal = constrain(sensitivity, 1.0f, 10.0f);
}

void MotionDetector::setDebugMode(bool enable) {
    _config.dbgMotion = enable;
}

void MotionDetector::setColorDepth(bool useColor) {
    _config.depthColor = useColor;
    _colorDepth = useColor ? RGB888_BYTES : GRAYSCALE_BYTES;
    _stride = (_colorDepth == RGB888_BYTES) ? GRAYSCALE_BYTES : RGB888_BYTES;
}

void MotionDetector::reset() {
    _motionCnt = 0;
    _nightCnt = 0;
    _motionJpegLen = 0;
    
    // Clear buffers
    if (_currBuff) memset(_currBuff, 0, RESIZE_DIM_SQ * RGB888_BYTES);
    if (_prevBuff) memset(_prevBuff, 0, RESIZE_DIM_SQ * RGB888_BYTES);
    if (_changeMap) memset(_changeMap, 0, RESIZE_DIM_SQ * RGB888_BYTES);
}

MotionDetector::Result MotionDetector::checkMotion(camera_fb_t* fb, bool isCapturing) {
    Result result = {false, 0, false, 0, 0, 0};
    uint32_t startTime = millis();
    
    if (!fb || !_currBuff) {
        return result;
    }
    
    // Calculate scale parameters
    uint8_t scaling = 2; // Default scaling factor
    uint16_t reducer = 4;
    uint8_t downsize = pow(2, scaling) * reducer;
    int sampleWidth = 800 / downsize;  // Adjust based on actual frame size
    int sampleHeight = 600 / downsize;
    
    // JPEG to RGB conversion
    if (!fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, _rgbBuf)) {
        return result;
    }
    
    if (_colorDepth == GRAYSCALE_BYTES) {
        rgbToGray(_rgbBuf, sampleWidth, sampleHeight);
    }
    
    // Resize to 96x96
    size_t resizeDimLen = RESIZE_DIM_SQ * _colorDepth;
    rescaleImage(_rgbBuf, sampleWidth, sampleHeight, _currBuff, RESIZE_DIM, RESIZE_DIM);
    
    // Compare frames and count changes
    uint16_t startPixel = (RESIZE_DIM * (_config.detectStartBand - 1) / _config.detectNumBands) * RESIZE_DIM * _colorDepth;
    uint16_t endPixel = (RESIZE_DIM * _config.detectEndBand / _config.detectNumBands) * RESIZE_DIM * _colorDepth;
    
    int changeCount = compareFrames(resizeDimLen, &startPixel, &endPixel);
    int threshold = calculateThreshold(startPixel, endPixel);
    
    // Update results
    result.changeCount = changeCount;
    result.threshold = threshold;
    result.lightLevel = _lightLevel;
    result.isNightTime = _nightTime;
    
    // Save current frame for next comparison
    memcpy(_prevBuff, _currBuff, resizeDimLen);
    
    // Motion validation
    if (_config.dbgMotion) {
        generateDebugJpeg();
    } else {
        result.motionDetected = validateMotion(changeCount, threshold, isCapturing);
    }
    
    result.processingTime = millis() - startTime;
    return result;
}

uint8_t MotionDetector::getLightLevel(camera_fb_t* fb) {
    Result result = checkMotion(fb, false);
    return result.lightLevel;
}

bool MotionDetector::isNight() {
    return _nightTime;
}

bool MotionDetector::getDebugJpeg(uint8_t** jpegBuf, size_t* jpegLen) {
    if (_motionJpegLen > 0 && _motionJpeg) {
        *jpegBuf = _motionJpeg;
        *jpegLen = _motionJpegLen;
        _motionJpegLen = 0; // Reset for next frame
        return true;
    }
    return false;
}

// Private methods

void MotionDetector::allocateBuffers() {
    if (!_currBuff) _currBuff = (uint8_t*)ps_malloc(RESIZE_DIM_SQ * RGB888_BYTES);
    if (!_prevBuff) _prevBuff = (uint8_t*)ps_malloc(RESIZE_DIM_SQ * RGB888_BYTES);
    if (!_changeMap) _changeMap = (uint8_t*)ps_malloc(RESIZE_DIM_SQ * RGB888_BYTES);
    if (!_rgbBuf) _rgbBuf = (uint8_t*)ps_malloc(800 * 600 * RGB888_BYTES / 8);
    if (!_motionJpeg) _motionJpeg = (uint8_t*)ps_malloc(32 * 1024);
}

void MotionDetector::freeBuffers() {
    if (_currBuff) { free(_currBuff); _currBuff = nullptr; }
    if (_prevBuff) { free(_prevBuff); _prevBuff = nullptr; }
    if (_changeMap) { free(_changeMap); _changeMap = nullptr; }
    if (_rgbBuf) { free(_rgbBuf); _rgbBuf = nullptr; }
    if (_motionJpeg) { free(_motionJpeg); _motionJpeg = nullptr; }
}

void MotionDetector::rescaleImage(const uint8_t* input, int inputWidth, int inputHeight, 
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

void MotionDetector::rgbToGray(uint8_t* buffer, int width, int height) {
    for (int i = 0; i < width * height; ++i) {
        int index = i * 3;
        buffer[i] = (uint8_t)(((77 * buffer[index]) + (150 * buffer[index + 1]) + (29 * buffer[index + 2])) >> 8);
    }
}

int MotionDetector::compareFrames(size_t resizeDimLen, uint16_t* startPixel, uint16_t* endPixel) {
    int changeCount = 0;
    uint32_t lux = 0;
    
    for (int i = 0; i < resizeDimLen; i += _colorDepth) {
        uint16_t currPix = 0, prevPix = 0;
        
        for (int j = 0; j < _colorDepth; j++) {
            currPix += _currBuff[i + j];
            prevPix += _prevBuff[i + j];
        }
        
        currPix /= _colorDepth;
        prevPix /= _colorDepth;
        lux += currPix;
        
        uint8_t pixVal = 255;
        
        if (_config.dbgMotion) {
            for (int j = 0; j < RGB888_BYTES; j++) {
                _changeMap[(i * _stride) + j] = currPix;
            }
        }
        
        if (abs((int)currPix - (int)prevPix) > _config.detectChangeThreshold) {
            if (i > *startPixel && i < *endPixel) {
                changeCount++;
            } else {
                pixVal = 80;
            }
            
            if (_config.dbgMotion) {
                _changeMap[(i * _stride) + 2] = pixVal;
                for (int j = 0; j < RGB888_BYTES - 1; j++) {
                    _changeMap[(i * _stride) + j] = 0;
                }
            }
        }
    }
    
    updateLightLevel(lux);
    return changeCount;
}

void MotionDetector::updateLightLevel(uint32_t lux) {
    _lightLevel = (lux * 100) / (RESIZE_DIM_SQ * 255);
    
    if (_nightTime) {
        if (_lightLevel > _config.nightSwitch) {
            _nightCnt--;
            if (_nightCnt <= 0) {
                _nightTime = false;
                _nightCnt = 0;
            }
        }
    } else {
        if (_lightLevel < _config.nightSwitch) {
            _nightCnt++;
            if (_nightCnt > _config.detectNightFrames) {
                _nightTime = true;
            }
        } else {
            _nightCnt = 0;
        }
    }
}

int MotionDetector::calculateThreshold(uint16_t startPixel, uint16_t endPixel) {
    return ((endPixel - startPixel) / _colorDepth) * (11 - _config.motionVal) / 100;
}

bool MotionDetector::validateMotion(int changeCount, int threshold, bool motionStatus) {
    if (_nightTime) {
        return false;
    }
    
    if (changeCount > threshold) {
        _motionCnt++;
        
        if (!motionStatus && _motionCnt >= _config.detectMotionFrames) {
            return true; // Motion started
        }
        
        if (motionStatus) {
            return true; // Motion ongoing
        }
    } else {
        _motionCnt = 0;
        return false;
    }
    
    return motionStatus;
}

bool MotionDetector::generateDebugJpeg() {
    if (_motionJpegLen == 0) {
        uint8_t* jpg_buf = NULL;
        size_t resizeDimLen = RESIZE_DIM_SQ * _colorDepth;
        
        if (fmt2jpg(_changeMap, resizeDimLen, RESIZE_DIM, RESIZE_DIM,
                    PIXFORMAT_RGB888, JPEG_QUAL, &jpg_buf, &_motionJpegLen)) {
            memcpy(_motionJpeg, jpg_buf, _motionJpegLen);
            free(jpg_buf);
            return true;
        }
    }
    return false;
}

bool MotionDetector::jpg2rgb(const uint8_t* src, size_t src_len, uint8_t* out, uint8_t scale) {
    // Using fmt2rgb888 from img_converters.h
    // Note: scale parameter is ignored in this version
    return fmt2rgb888(src, src_len, PIXFORMAT_JPEG, out);
}

MotionDetector::BoundingBox MotionDetector::getMotionBoundingBox() {
    return extractBoundingBox();
}

MotionDetector::BoundingBox MotionDetector::extractBoundingBox() {
    BoundingBox bbox = {false, 0, 0, 0, 0, 0, 0, RESIZE_DIM, RESIZE_DIM};
    
    if (!_changeMap) {
        return bbox;
    }
    
    uint16_t minX = RESIZE_DIM;
    uint16_t minY = RESIZE_DIM;
    uint16_t maxX = 0;
    uint16_t maxY = 0;
    uint32_t sumX = 0;
    uint32_t sumY = 0;
    uint32_t count = 0;
    
    // Scan changeMap for red pixels (motion markers)
    // changeMap is RGB888 format, red channel is at index 2
    for (int y = 0; y < RESIZE_DIM; ++y) {
        for (int x = 0; x < RESIZE_DIM; ++x) {
            int pixelIdx = (y * RESIZE_DIM + x) * RGB888_BYTES;
            
            // Check if red channel has significant value (motion detected)
            // Red pixels have value 255 for motion in ROI, 80 for motion outside ROI
            uint8_t redValue = _changeMap[pixelIdx + 2];
            
            if (redValue >= 80) {  // Detect any motion marker
                // Update bounding box
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
                
                // Accumulate for centroid calculation (weighted by red intensity)
                sumX += x * redValue;
                sumY += y * redValue;
                count += redValue;
            }
        }
    }
    
    // Check if we found any motion
    if (count > 0 && maxX >= minX && maxY >= minY) {
        bbox.valid = true;
        bbox.x1 = minX;
        bbox.y1 = minY;
        bbox.x2 = maxX;
        bbox.y2 = maxY;
        
        // Calculate weighted centroid
        bbox.centerX = sumX / count;
        bbox.centerY = sumY / count;
        
        // Expand bbox slightly for better coverage (5% padding)
        uint16_t padX = max(1, (maxX - minX) / 20);
        uint16_t padY = max(1, (maxY - minY) / 20);
        
        bbox.x1 = (minX > padX) ? minX - padX : 0;
        bbox.y1 = (minY > padY) ? minY - padY : 0;
        bbox.x2 = min((uint16_t)(maxX + padX), (uint16_t)(RESIZE_DIM - 1));
        bbox.y2 = min((uint16_t)(maxY + padY), (uint16_t)(RESIZE_DIM - 1));
    }
    
    return bbox;
}
