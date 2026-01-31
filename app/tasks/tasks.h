#pragma once

#include <Arduino.h>
#include <main.h>
#include <vector>
#include <portmacro.h>
#include <WebServer.h>
#include <FTPServer.h>
#include <map>

typedef void (*SomeTask)(void* param);

struct BackgroundTask {
	const char* name;
	TaskHandle_t handle = nullptr;
	SomeTask task;
	uint32_t stack = 4096;
	BaseType_t core = 1;
	UBaseType_t priority = 0;
	UBaseType_t caps = MALLOC_CAP_INTERNAL;
	bool suspendable = true;
};

extern FTPServer ftpServer;
std::map<int, BackgroundTask*> getTasks();

void createTask(BackgroundTask* task);
void resumeTasks();
void pauseTasks();
void runTasks();
eTaskState getTaskStatus(BackgroundTask* task);
void restartTask(BackgroundTask* task);

void networkTask(void *param);
void handlerTask(void* param);
void monitorerTask(void* param);