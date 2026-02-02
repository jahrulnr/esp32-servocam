# ESP32 ServoCam

ESP32 ServoCam is an embedded camera application for ESP32-class devices that provides camera streaming, servo control, and a Docker-based backend for image inference (YOLO). The repository contains the PlatformIO firmware, local libraries, static web assets, and a Docker Compose configuration for backend services.

Key features
- Camera streaming and JPEG snapshots
- Servo control for X/Y axes via WebSocket and HTTP APIs
- Backend image inference via YOLO (Docker)

Repository layout (important files)
- `platformio.ini`: PlatformIO project configuration and build environments (`esp32cam`, `esp32s3dev`)
- `app/`: firmware source (entry point: `app/main.cpp`)
- `include/`: project headers and config templates (see `include/Secret.h.example`)
- `lib/`: local libraries (e.g., `ServoControl`, `WiFiManager`)
- `data/`: static web assets (UI)
- `backend/compose.yml`: Docker Compose configuration for backend services

Prerequisites
- PlatformIO (recommended via VSCode + PlatformIO extension)
- Docker and Docker Compose for backend services

Building and uploading firmware
1. Connect the ESP32-CAM to your host via a USB-to-Serial adapter. Ensure the appropriate flash-mode wiring (IO0) when flashing.
2. Verify the target environment in `platformio.ini` (e.g. `[env:esp32cam]`).
3. Build and flash using PlatformIO:

```bash
cd /path/to/esp32-servocam
platformio run -e esp32cam
platformio run -e esp32cam -t upload
```

If you use VSCode with PlatformIO, use the IDE’s Build and Upload actions.

Firmware configuration
- Copy `include/Secret.h.example` to `include/Secret.h` and populate network credentials and other secrets.
- Review `include/CameraConfig.h` and `HW.h` for camera model and pin mappings (AI-Thinker vs. ESP32-S3 variants).

Running the backend locally
1. From the `backend/` directory, start services:

```bash
cd backend
docker compose up --build -d
```

2. Primary services
- Gateway: exposed on port 8080 (API and backend UI)
- YOLO inference service: exposed on port 5000

Compose configuration: [backend/compose.yml](backend/compose.yml)

Web UI and assets
- Static UI assets are located in the `data/` directory (`data/index.html`). These can be served by the gateway or hosted separately.

Debugging and logs
- Serial monitor: `platformio device monitor -e esp32cam` or use the PlatformIO Serial Monitor in VSCode.
- When PSRAM is enabled, camera frame allocation will prefer PSRAM (see `platformio.ini` build_flags and `app/main.cpp`).

Development notes
- Main firmware is under `app/`; background tasks live in `app/tasks/`.
- Servo logic is implemented in `lib/ServoControl` with callbacks in `app/callback/servo.h`.
- Add additional library dependencies via `lib_deps` in `platformio.ini`.

Additional notes
- `platformio.ini` includes environments for both `esp32cam` and `esp32s3dev`.
- Custom partition tables are provided in `boards/*.csv` if required by your board.
