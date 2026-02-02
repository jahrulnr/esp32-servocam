#pragma once
#include <SpiJsonDocument.h>
#include <ServoControl.h>
#include <WebSocketsServer.h>

extern ServoControl* xServo;
extern ServoControl* yServo;
extern WebSocketsServer wsServer;

// Broadcast servo updates to connected WebSocket clients. Uses same message
// `type` as router's ack to avoid duplication: `servo_ack`.
inline void servoUpdateX() {
	SpiJsonDocument resp;
	resp["type"] = "servo_ack";
	resp["axis"] = "x";
	resp["angle"] = xServo ? xServo->getAngle() : 0;
	String out;
	serializeJson(resp, out);
	wsServer.broadcastTXT(out);
}

inline void servoUpdateY() {
	SpiJsonDocument resp;
	resp["type"] = "servo_ack";
	resp["axis"] = "y";
	resp["angle"] = yServo ? yServo->getAngle() : 0;
	String out;
	serializeJson(resp, out);
	wsServer.broadcastTXT(out);
}
