#include "DetectionHandler.h"
#include <Arduino.h>

void handleDetections(ObjectDetectionService &objectDetectionService, WebSocketsServer &wsServer, ServoControl* xServo, ServoControl* yServo) {
    if (!objectDetectionService.isReady()) return;

    auto detectionResult = objectDetectionService.getResult();
    SpiJsonDocument detectionJson;
    detectionJson["type"] = "detections";
    detectionJson["width"] = detectionResult.width;
    detectionJson["height"] = detectionResult.height;
    JsonArray detectionsArray = detectionJson.createNestedArray("detections");

    // Find detection with highest (possibly adjusted) confidence
    float bestConf = -1.0f;
    int bestIdx = -1;
    for (size_t i = 0; i < detectionResult.detections.size(); ++i) {
        auto &det = detectionResult.detections[i];
        float conf = det.confidence;
        if (det.classification == "person") {
            conf += 0.2f; // khusus person, tambahkan 0.2
            if (conf > 1.0f) conf = 1.0f;
        }
        if (conf > bestConf) {
            bestConf = conf;
            bestIdx = i;
        }

        // add to JSON using adjusted confidence for person
        SpiJsonDocument obj;
        JsonArray box = obj.createNestedArray("box");
        box.add(det.x1);
        box.add(det.y1);
        box.add(det.x2);
        box.add(det.y2);
        obj["confidence"] = (det.classification == "person") ? min(1.0f, det.confidence + 0.2f) : det.confidence;
        obj["oid"] = det.oid;
        obj["classification"] = det.classification;
        detectionsArray.add(obj);
    }

    // If we found a best detection, move servos to its center
    if (bestIdx >= 0 && bestIdx < (int)detectionResult.detections.size()) {
        auto &best = detectionResult.detections[bestIdx];
        float centerX = (best.x1 + best.x2) * 0.5f;
        float centerY = (best.y1 + best.y2) * 0.5f;
        int angleX = 90; // fallback
        int angleY = 90;
        if (detectionResult.width > 0) {
            // map image X -> servo angle so that low X -> right, high X -> left
            angleX = (int)round((1.0f - (centerX / detectionResult.width)) * 180.0f);
            angleX = constrain(angleX, 0, 180);
        }
        if (detectionResult.height > 0) {
            angleY = (int)round((centerY / detectionResult.height) * 180.0f);
            angleY = constrain(angleY, 0, 180);
        }

        // Smooth incremental movement: compute delta from current servo angle
        const int MAX_STEP = 8; // max degrees change per detection update
        const int MIN_STEP = 2; // deadzone - ignore smaller changes

        // Additional center deadzone (in pixels) to avoid oscillation when object is near center
        const float CENTER_DEAD_FRAC = 0.04f; // 4% of image dimension
        const int centerDeadX = (int)(detectionResult.width * CENTER_DEAD_FRAC);
        const int centerDeadY = (int)(detectionResult.height * CENTER_DEAD_FRAC);

        // Ignore movement if bounding box is too close to edges
        const float EDGE_FRAC = 0.03f; // 3% margin
        const int edgeMarginY = (int)(detectionResult.height * EDGE_FRAC);
        const int edgeMarginX = (int)(detectionResult.width * EDGE_FRAC);

        bool ignoreYDueToEdge = false;
        int boxH = (int)(best.y2 - best.y1);
        if ((best.y1 <= edgeMarginY && best.y2 >= (detectionResult.height - edgeMarginY))
            || (boxH >= detectionResult.height - 2 * edgeMarginY)) {
            ignoreYDueToEdge = true;
        }

        bool ignoreXDueToEdge = false;
        int boxW = (int)(best.x2 - best.x1);
        if ((best.x1 <= edgeMarginX && best.x2 >= (detectionResult.width - edgeMarginX))
            || (boxW >= detectionResult.width - 2 * edgeMarginX)) {
            ignoreXDueToEdge = true;
        }

        if (xServo != nullptr) {
            int curX = xServo->getAngle();
            int imgCenterX = detectionResult.width / 2;
            if (!(ignoreXDueToEdge || abs((int)centerX - imgCenterX) <= centerDeadX)) {
                int d = angleX - curX;
                if (abs(d) >= MIN_STEP) {
                    // Set full target and let ServoControl do smooth movement
                    xServo->setAngle(angleX);
                }
            }
        }

        if (yServo != nullptr) {
            int curY = yServo->getAngle();
            int imgCenterY = detectionResult.height / 2;
            if (!ignoreYDueToEdge && !(abs((int)centerY - imgCenterY) <= centerDeadY)) {
                int d = angleY - curY;
                if (abs(d) >= MIN_STEP) {
                    // Set full target and let ServoControl do smooth movement
                    yServo->setAngle(angleY);
                }
            }
        }
    }

    String outTxt;
    serializeJson(detectionJson, outTxt);
    wsServer.broadcastTXT(outTxt);
}
