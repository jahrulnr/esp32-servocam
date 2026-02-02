#pragma once

#include <Config.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <SpiJsonDocument.h>
#include <main.h>

extern WebServer server;
extern WebSocketsServer wsServer;
extern ServoControl* xServo;
extern ServoControl* yServo;

static int clientNum = 0;
inline int wsClientsNum() { return clientNum; }

// WebSocket event handler
inline void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      --clientNum;
      break;
    case WStype_CONNECTED: {
      IPAddress ip = wsServer.remoteIP(num);
      ++clientNum;
      Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      // Send welcome message
      wsServer.sendTXT(num, "Connected to ESP-AI WebSocket Server");
    }
      break;
    case WStype_TEXT:
      {
          SpiJsonDocument body;
          deserializeJson(body, payload);

          if (body["type"] == "ping") {
            return;
          }

          // Servo control message: { type: "servo", axis: "x"|"y", angle: <0-180> }
          if (body["type"] == "servo") {
            String axis = body["axis"] | "";
            int angle = body["angle"] | -1;
            if (angle >= 0) {
              if (axis == "x") {
                if (xServo != nullptr) xServo->setAngle(angle);
              } else if (axis == "y") {
                if (yServo != nullptr) yServo->setAngle(angle);
              }

              // Ack will be broadcast from servo updater callback to avoid duplicates
            }
            return;
          }
      }
      break;
    case WStype_BIN:
      Serial.printf("[%u] get binary length: %u\n", num, length);
      // Handle binary data if needed
      break;
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    case WStype_PING:
    case WStype_PONG:
      break;
  }
}

// WebSocket methods
inline void sendMessage(String message) {
  wsServer.broadcastTXT(message);
}

inline void sendMessageToClient(uint8_t num, String message) {
  wsServer.sendTXT(num, message);
}

inline int getConnectedClients() {
  return wsServer.connectedClients();
}

inline void disconnectClient(uint8_t num) {
  wsServer.disconnect(num);
}

inline void setupWebRouter() {
  server.serveStatic("/", STORAGE, "/index.html");
  server.serveStatic("/favicon.ico", STORAGE, "/favicon.ico");
  server.serveStatic("/assets", STORAGE, "/assets/");
  server.begin();

  wsServer.onEvent(onWsEvent);
  wsServer.begin();
}