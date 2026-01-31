#include "tasks.h"

std::map<int, BackgroundTask*> tasks;

void runTasks(){
	createTask(new BackgroundTask{
		.name = "networkTask",
		.task = networkTask,
		.stack = 4 * 1024,
		.core = 1,
		.priority = 0,
		.suspendable = true,
	});
	createTask(new BackgroundTask{
		.name = "handlerTask",
		.task = handlerTask,
		.stack = 3 * 1024,
		.core = tskNO_AFFINITY,
		.priority = 6,
		.suspendable = true,
	});
	createTask(new BackgroundTask{
		.name = "monitorerTask",
		.task = monitorerTask,
		.stack = 5 * 1024,
		.core = tskNO_AFFINITY,
		.priority = 0,
		.suspendable = false,
	});
}

std::map<int, BackgroundTask*> getTasks() {
	return tasks;
}

void createTask(BackgroundTask* task){
	for(auto item: tasks) {
		if (item.second == task) return;
	}

	static int taskId = 0;
	tasks[taskId++] = task;
	xTaskCreatePinnedToCore(
		task->task,
		task->name,
		task->stack,
		NULL,
		task->priority,
		&task->handle,
		task->core
	);
}

void resumeTasks(){
	if(tasks.size() == 0) return;

	for(auto task: tasks) {
		if (task.second->suspendable && task.second->handle != nullptr) {
			vTaskResume(task.second->handle);
		}
	}
}

void pauseTasks(){
	if(tasks.size() == 0) return;
	
	for(auto task: tasks) {
		if (task.second->suspendable && task.second->handle != nullptr) {
			vTaskSuspend(task.second->handle);
		}
	}
}

eTaskState getTaskStatus(BackgroundTask* task) {
    if (task == nullptr || task->handle == nullptr) {
        return eDeleted;
    }
    return eTaskGetState(task->handle);
}

void restartTask(BackgroundTask* task) {
    if (task == nullptr) return;

    // Kalau masih hidup → matikan dulu
    if (task->handle != nullptr) {
        vTaskDelete(task->handle);
        task->handle = nullptr;
    }

    xTaskCreatePinnedToCore(
        task->task,
        task->name,
        task->stack,
        NULL,
        task->priority,
        &task->handle,
        task->core
    );
}
