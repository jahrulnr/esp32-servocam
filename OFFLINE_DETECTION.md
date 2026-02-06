# Offline Detection using Motion Tracking

## Overview

`OfflineDetectionService` provides object detection capabilities using motion detection when network/backend is unavailable. It uses `MotionDetector` to track moving objects and generates bounding boxes compatible with `ObjectDetectionService`.

## How It Works

1. **Motion Detection**: Compares consecutive frames to detect pixel changes
2. **Edge Extraction**: Identifies red markers in changeMap (motion regions)
3. **Bounding Box Calculation**: Finds min/max coordinates of motion pixels
4. **Centroid Calculation**: Weighted average of motion pixels for accurate center
5. **Coordinate Scaling**: Scales from 96x96 to camera resolution (800x600)

## Features

✅ **Full Offline Operation** - No network required  
✅ **ObjectDetectionService Compatible** - Drop-in replacement  
✅ **Bounding Box Support** - Returns x1, y1, x2, y2 coordinates  
✅ **Center Point Calculation** - For servo tracking  
✅ **Configurable Sensitivity** - Adjustable motion threshold  
✅ **Area Filtering** - Ignore small/noise motion  

## Usage Example

```cpp
#include "service/OfflineDetectionService.h"

OfflineDetectionService offlineDetection;

void setup() {
    // Configure motion detector
    MotionDetector::Config config;
    config.motionVal = 8.0;              // Sensitivity (1-10)
    config.detectMotionFrames = 3;       // Min frames to confirm
    config.detectChangeThreshold = 15;   // Pixel diff threshold
    config.depthColor = false;           // Grayscale mode
    config.dbgMotion = false;            // Disable debug vis
    
    offlineDetection.init(config);
    offlineDetection.setMinMotionArea(100);  // Min 100 pixels
}

void loop() {
    camera_fb_t* fb = esp_camera_fb_get();
    
    // Feed frame
    offlineDetection.feed(fb);
    
    // Check if detection ready
    if (offlineDetection.isReady()) {
        auto result = offlineDetection.getResult();
        
        // Use with DetectionHandler (same interface as ObjectDetectionService)
        for (auto& det : result.detections) {
            Serial.printf("Motion at: [%d,%d,%d,%d] conf=%.2f\n",
                         det.x1, det.y1, det.x2, det.y2, det.confidence);
            
            // Calculate center for servo
            float centerX = (det.x1 + det.x2) / 2.0f;
            float centerY = (det.y1 + det.y2) / 2.0f;
        }
    }
    
    esp_camera_fb_return(fb);
}
```

## Integration with DetectionHandler

```cpp
// Option 1: Replace ObjectDetectionService
OfflineDetectionService detectionService;
handleDetections(detectionService, wsServer, xServo, yServo);

// Option 2: Hybrid mode (fallback to offline)
if (networkAvailable) {
    handleDetections(onlineService, wsServer, xServo, yServo);
} else {
    handleDetections(offlineService, wsServer, xServo, yServo);
}
```

## Configuration Parameters

### MotionDetector Config
- `motionVal` (1-10): Motion sensitivity (higher = more sensitive)
- `detectMotionFrames` (int): Min consecutive frames to confirm motion
- `detectChangeThreshold` (0-255): Pixel difference threshold
- `depthColor` (bool): false=grayscale (faster), true=RGB
- `detectStartBand` / `detectEndBand`: ROI vertical bands (0-10)
- `nightSwitch` (%): Light level threshold for night mode

### OfflineDetectionService Config
- `setMinMotionArea(pixels)`: Minimum bbox area to report (default: 100)
- `setMotionSensitivity(1-10)`: Wrapper for motionVal

## Detection Result Format

```cpp
struct Detection {
    int x1, y1, x2, y2;      // Bounding box (scaled to camera res)
    float confidence;         // 0.5-0.95 (based on motion area)
    int oid;                  // Always 0 for motion
    String classification;    // Always "motion"
};

struct ObjectDetectionResult {
    bool success;
    std::vector<Detection> detections;
    uint16_t width;          // Camera width (800)
    uint16_t height;         // Camera height (600)
    String error;
};
```

## Bounding Box Extraction Algorithm

1. **Scan changeMap** (96x96 RGB888):
   - Red channel ≥ 80 = motion pixel
   - Track min/max X/Y coordinates

2. **Calculate weighted centroid**:
   - Weight each pixel by red intensity (80 or 255)
   - centerX = Σ(x × red) / Σ(red)
   - centerY = Σ(y × red) / Σ(red)

3. **Expand bbox** (5% padding):
   - Adds margin around detected region
   - Prevents edge clipping

4. **Scale to camera resolution**:
   - x_cam = x_96 × (800 / 96)
   - y_cam = y_96 × (600 / 96)

## Limitations vs Full Object Detection

| Feature | OfflineDetection | ObjectDetectionService |
|---------|------------------|------------------------|
| Network Required | ❌ No | ✅ Yes |
| Multi-object | ❌ Single bbox | ✅ Multiple |
| Classification | ❌ Generic "motion" | ✅ Person, cat, dog, etc |
| Bounding Box | ✅ Yes | ✅ Yes |
| Confidence | ⚠️ Area-based | ✅ ML confidence |
| Speed | ✅ Fast (~50ms) | ⚠️ Slower (~200ms+) |
| Memory | ✅ Low | ⚠️ Higher |

## Performance

- **Processing Time**: ~50-100ms per frame
- **Memory Usage**: ~500KB PSRAM (motion buffers)
- **Accuracy**: Depends on motion visibility and lighting
- **Best For**: 
  - Moving objects
  - Well-lit environments
  - Single dominant object

## Troubleshooting

**No detection despite movement**:
- Increase `motionVal` (sensitivity)
- Decrease `detectChangeThreshold`
- Check `detectStartBand` / `detectEndBand` ROI

**Too many false positives**:
- Decrease `motionVal`
- Increase `setMinMotionArea()`
- Increase `detectMotionFrames`

**Bbox too small/large**:
- Adjust `detectChangeThreshold`
- Check lighting (nightSwitch threshold)

**Night mode blocking detection**:
- Set `config.nightSwitch = 0` to disable

## Example: Hybrid Online/Offline System

```cpp
#define USE_OFFLINE_FALLBACK true

ObjectDetectionService onlineService;
OfflineDetectionService offlineService;

void setup() {
    onlineService.init("http://backend:8000/detect");
    
    MotionDetector::Config cfg;
    cfg.motionVal = 7.5;
    offlineService.init(cfg);
}

void loop() {
    camera_fb_t* fb = esp_camera_fb_get();
    
    #if USE_OFFLINE_FALLBACK
    if (WiFi.status() == WL_CONNECTED) {
        onlineService.feed(fb->buf, fb->len);
        if (onlineService.isReady()) {
            handleDetections(onlineService, wsServer, xServo, yServo);
        }
    } else {
        offlineService.feed(fb);
        if (offlineService.isReady()) {
            handleDetections(offlineService, wsServer, xServo, yServo);
        }
    }
    #endif
    
    esp_camera_fb_return(fb);
}
```
