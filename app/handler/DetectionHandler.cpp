#include "DetectionHandler.h"
#include <Arduino.h>
#include "Config.h"

void handleDetections(ObjectDetectionService &objectDetectionService, WebSocketsServer &wsServer, ServoControl* xServo, ServoControl* yServo) {
    // Allow awareness/search even when detector isn't ready.
    ObjectDetectionService::ObjectDetectionResult detectionResult;
    if (objectDetectionService.isReady()) {
        detectionResult = objectDetectionService.getResult();
    } else {
        // leave detectionResult empty (width/height == 0)
        // awareness maps grid directly to servo range when dims unavailable
    }
    SpiJsonDocument detectionJson;
    detectionJson["type"] = "detections";
    detectionJson["width"] = detectionResult.width;
    detectionJson["height"] = detectionResult.height;
    JsonArray detectionsArray = detectionJson.createNestedArray("detections");

    // Find detection with highest (possibly adjusted) confidence
    float bestConf = -1.0f;
    int bestIdx = -1;
#if ENABLE_OBJECT_DETECTION
    for (size_t i = 0; i < detectionResult.detections.size(); ++i) {
        auto &det = detectionResult.detections[i];
        float conf = det.confidence;
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
#else
    // object detection disabled for testing — leave detectionsArray empty and force search
    bestIdx = -1;
#endif

    // If we found a best detection, optionally use RL to select an action and move servos
    if (bestIdx >= 0 && bestIdx < (int)detectionResult.detections.size()) {
        auto &best = detectionResult.detections[bestIdx];
        float centerX = (best.x1 + best.x2) * 0.5f;
        float centerY = (best.y1 + best.y2) * 0.5f;
        int angleX = 90; // fallback
        int angleY = 90;

        // compute error in "degrees" relative to center (for RL discretization)
        // use current center for error computation
        float errDegX = ((centerX - (detectionResult.width / 2.0f)) / (float)max(1, detectionResult.width)) * 180.0f;
        float errDegY = ((centerY - (detectionResult.height / 2.0f)) / (float)max(1, detectionResult.height)) * 180.0f;
        float cur_err_mag = sqrt(errDegX * errDegX + errDegY * errDegY);


        // Additional center deadzone (in pixels) to avoid oscillation when object is near center
        const float CENTER_DEAD_FRAC = 0.08f; // % of image dimension
        const int centerDeadX = (int)(detectionResult.width * CENTER_DEAD_FRAC);
        int centerDeadY = (int)(detectionResult.height * CENTER_DEAD_FRAC);

        // Declare bbox size variables here so they are available later
        int boxW = 0;
        int boxH = 0;

        if (detectionResult.width > 0 && detectionResult.height > 0) {
            // Normalize image coordinates to 0..180 range for better precision
            // normalize center to 0..180 range
            float normX = (centerX / detectionResult.width) * 180.0f; // 0..180 left->right
            float normY = (centerY / detectionResult.height) * 180.0f; // 0..180 top->bottom

            // EMA smoothing to reduce jitter
            static float emaX = -1.0f;
            static float emaY = -1.0f;
            const float EMA_ALPHA = 0.45f;
            if (emaX < 0) emaX = normX; else emaX = EMA_ALPHA * normX + (1.0f - EMA_ALPHA) * emaX;
            if (emaY < 0) emaY = normY; else emaY = EMA_ALPHA * normY + (1.0f - EMA_ALPHA) * emaY;

            // For X, we want low image X -> right (high angle), high image X -> left (low angle)
            int rawAngleX = (int)round(180.0f - emaX);
            int rawAngleY = (int)round(emaY);
            // use raw target angle (EMA already smooths jitter). No partial-target reduction.
            angleX = rawAngleX;
            angleY = rawAngleY;

            // compute bbox size in pixels
            boxW = (int)(best.x2 - best.x1);
            boxH = (int)(best.y2 - best.y1);

            // Adjust movement based on object size to avoid over-movement for large objects
            // sizeFrac: fraction of image occupied by bbox (use max of width and height fraction)
            float sizeFracW = (float)boxW / (float)max(1, detectionResult.width);
            float sizeFracH = (float)boxH / (float)max(1, detectionResult.height);
            float sizeFrac = max(sizeFracW, sizeFracH); // 0..1

            // Map sizeFrac to a scale factor in [minScale..maxScale]
            // small objects -> scale ~ maxScale (move fully)
            // large objects -> scale ~ minScale (reduce movement)
            const float sizeMin = 0.05f; // 5% -> small
            const float sizeMax = 0.4f;  // 40% -> threshold for full centering
            const float maxScale = 1.0f;
            const float minScale = 0.6f; // do not reduce movement too much
            float scale;
            if (sizeFrac >= sizeMax) {
                // For very large objects, use full centering to move to center precisely
                scale = 1.0f;
            } else {
                float t = (sizeFrac - sizeMin) / (sizeMax - sizeMin);
                t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
                scale = maxScale - (maxScale - minScale) * t;
            }

            // Apply scale relative to center (90 deg) to reduce movement magnitude for large objects
            float relX = (float)angleX - 90.0f;
            float relY = (float)angleY - 90.0f;
            angleX = (int)round(90.0f + relX * scale);
            angleY = (int)round(90.0f + relY * scale);

            angleX = constrain(angleX, 0, 180);
            angleY = constrain(angleY, 0, 180);
        }
        // If object is small vertically, reduce the vertical deadzone so servo reacts
        // small object threshold: less than 8% of image height
        if ( (float)boxH / (float)max(1, detectionResult.height) < 0.08f) {
            centerDeadY = max(1, centerDeadY / 2);
        }

        // Simple proportional controller: compute target angles, but limit per-frame step
        int imgCenterX = detectionResult.width / 2;
        int imgCenterY = detectionResult.height / 2;
        bool inCenter = (abs((int)centerX - imgCenterX) <= centerDeadX) && (abs((int)centerY - imgCenterY) <= centerDeadY);

        const int MAX_STEP_DEG = 6;
        int curServoX = xServo ? xServo->getAngle() : 90;
        int curServoY = yServo ? yServo->getAngle() : 90;
        int targetX = curServoX;
        int targetY = curServoY;
        if (!inCenter) {
            // angleX/angleY computed from EMA map to desired absolute angles
            int dx = angleX - curServoX;
            if (abs(dx) > MAX_STEP_DEG) targetX = curServoX + (dx > 0 ? MAX_STEP_DEG : -MAX_STEP_DEG);
            else targetX = angleX;

            int dy = angleY - curServoY;
            if (abs(dy) > MAX_STEP_DEG) targetY = curServoY + (dy > 0 ? MAX_STEP_DEG : -MAX_STEP_DEG);
            else targetY = angleY;
        }

        // Ignore movement if bounding box is too close to edges
        const float EDGE_FRAC = 0.03f; // 3% margin
        const int edgeMarginY = (int)(detectionResult.height * EDGE_FRAC);
        const int edgeMarginX = (int)(detectionResult.width * EDGE_FRAC);

        bool ignoreYDueToEdge = false;
        if ((best.y1 <= edgeMarginY && best.y2 >= (detectionResult.height - edgeMarginY))
            || (boxH >= detectionResult.height - 2 * edgeMarginY)) {
            ignoreYDueToEdge = true;
        }

        bool ignoreXDueToEdge = false;
        if ((best.x1 <= edgeMarginX && best.x2 >= (detectionResult.width - edgeMarginX))
            || (boxW >= detectionResult.width - 2 * edgeMarginX)) {
            ignoreXDueToEdge = true;
        }

        // Hysteresis: only change target if it differs enough
        const int HYSTERESIS_DEG = 3;
        if (xServo != nullptr) {
            static int prevTargetX = -999;
            int curX = xServo->getAngle();
            int imgCenterX = detectionResult.width / 2;
            if (!(ignoreXDueToEdge || abs((int)centerX - imgCenterX) <= centerDeadX)) {
                if (prevTargetX < -900 || abs(targetX - prevTargetX) >= HYSTERESIS_DEG) {
                    xServo->setAngle(targetX);
                    prevTargetX = targetX;
                }
            }
        }

        if (yServo != nullptr) {
            static int prevTargetY = -999;
            int curY = yServo->getAngle();
            int imgCenterY = detectionResult.height / 2;
            if (!ignoreYDueToEdge && !(abs((int)centerY - imgCenterY) <= centerDeadY)) {
                if (prevTargetY < -900 || abs(targetY - prevTargetY) >= HYSTERESIS_DEG) {
                    yServo->setAngle(targetY);
                    prevTargetY = targetY;
                }
            }
        }
    }

    // Awareness: only run grid search if enabled AND (object detection disabled OR 30s idle)
#if ENABLE_AWARENESS_CONTEXT
    static unsigned long lastDetectionT = 0; // updated when we have a detection
    // update lastDetectionT when detection present
    if (bestIdx >= 0) {
        lastDetectionT = millis();
    }

    bool allowAwareness = false;
    unsigned long now_check = millis();
#if ENABLE_OBJECT_DETECTION
    // If we have never seen any detection yet, allow awareness immediately.
    if (lastDetectionT == 0) {
        allowAwareness = true;
    } else if (now_check - lastDetectionT >= 30000UL) {
        allowAwareness = true;
    }
#else
    // object detection disabled -> allow awareness immediately
    allowAwareness = true;
#endif

    if (bestIdx < 0 && allowAwareness) {
        static int searchRow = 0, searchCol = 0;
        static unsigned long lastSearchT = 0;
        unsigned long now = millis();
        if (lastSearchT == 0 || now - lastSearchT >= AWARENESS_GRID_DWELL_MS) {
            // advance to next cell
            searchCol++;
            if (searchCol >= AWARENESS_GRID_COLS) {
                searchCol = 0;
                searchRow++;
                if (searchRow >= AWARENESS_GRID_ROWS) searchRow = 0;
            }
            lastSearchT = now;
        }
        // map grid cell to servo angles. If image dims available, map via image center
        int angleX = 90;
        int angleY = 90;
        // Map grid cell to servo angles. Use edge-to-edge mapping so the grid
        // covers the full 0..180 servo range. If only one row/col is configured
        // keep center behavior to avoid division by zero.
        if (detectionResult.width > 0 && detectionResult.height > 0) {
            float fracX;
            float fracY;
            if (AWARENESS_GRID_COLS > 1) fracX = (float)searchCol / (float)(AWARENESS_GRID_COLS - 1);
            else fracX = 0.5f;
            if (AWARENESS_GRID_ROWS > 1) fracY = (float)searchRow / (float)(AWARENESS_GRID_ROWS - 1);
            else fracY = 0.5f;
            float cellCenterX = fracX * detectionResult.width;
            float cellCenterY = fracY * detectionResult.height;
            angleX = (int)round(180.0f - (cellCenterX / detectionResult.width) * 180.0f);
            angleY = (int)round((cellCenterY / detectionResult.height) * 180.0f);
        } else {
            // no image dims: map columns/rows directly to full servo angle range 0..180
            float fracX;
            float fracY;
            if (AWARENESS_GRID_COLS > 1) fracX = (float)searchCol / (float)(AWARENESS_GRID_COLS - 1);
            else fracX = 0.5f;
            if (AWARENESS_GRID_ROWS > 1) fracY = (float)searchRow / (float)(AWARENESS_GRID_ROWS - 1);
            else fracY = 0.5f;
            angleX = (int)round(180.0f - fracX * 180.0f);
            angleY = (int)round(fracY * 180.0f);
        }
        angleX = constrain(angleX, 0, 180);
        angleY = constrain(angleY, 0, 180);

        // bounded-step movement while searching
        const int MAX_STEP_DEG_SEARCH = 8;
        int curServoX = xServo ? xServo->getAngle() : 90;
        int curServoY = yServo ? yServo->getAngle() : 90;
        int targetX = curServoX;
        int targetY = curServoY;
        int dx = angleX - curServoX;
        if (abs(dx) > MAX_STEP_DEG_SEARCH) targetX = curServoX + (dx > 0 ? MAX_STEP_DEG_SEARCH : -MAX_STEP_DEG_SEARCH);
        else targetX = angleX;
        int dy = angleY - curServoY;
        if (abs(dy) > MAX_STEP_DEG_SEARCH) targetY = curServoY + (dy > 0 ? MAX_STEP_DEG_SEARCH : -MAX_STEP_DEG_SEARCH);
        else targetY = angleY;

        // apply hysteresis when moving during search
        const int HYSTERESIS_DEG = 3;
        if (xServo != nullptr) {
            static int prevTargetXsearch = -999;
            if (prevTargetXsearch < -900 || abs(targetX - prevTargetXsearch) >= HYSTERESIS_DEG) {
                xServo->setAngle(targetX);
                prevTargetXsearch = targetX;
            }
        }
        if (yServo != nullptr) {
            static int prevTargetYsearch = -999;
            if (prevTargetYsearch < -900 || abs(targetY - prevTargetYsearch) >= HYSTERESIS_DEG) {
                yServo->setAngle(targetY);
                prevTargetYsearch = targetY;
            }
        }
    }
#endif

    if(detectionResult.width >0){
        String outTxt;
        serializeJson(detectionJson, outTxt);
        wsServer.broadcastTXT(outTxt);
    }
}
