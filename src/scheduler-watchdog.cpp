#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

struct TaskControl {
    TaskHandle_t taskHandle;
    uint32_t lastCheckIn;
    uint32_t timeout;
};

// Array of task controls
TaskControl taskControls[2];

void taskFunction(void *parameters) {
    int taskIndex = *(int*)parameters;
    while (true) {
        // Simulate task work
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
        taskControls[taskIndex].lastCheckIn = xTaskGetTickCount(); // Update check-in time
        Serial.println("Task running: " + String(taskIndex));
    }
}

void watchdogTask(void *parameters) {
    while (true) {
        for (int i = 0; i < 2; i++) {
            uint32_t currentTime = xTaskGetTickCount();
            if ((currentTime - taskControls[i].lastCheckIn) > taskControls[i].timeout) {
                Serial.println("Software Watchdog detected a problem with Task " + String(i));
                // Optionally, reset the task or take other remedial action here
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100 milliseconds
    }
}

void setup() {
    Serial.begin(115200);
    static int taskIndices[] = {0, 1}; // Indices to identify tasks

    // Initialize task control blocks
    for (int i = 0; i < 2; i++) {
        taskControls[i].timeout = 1500; // Set timeout to 1500 ms
        xTaskCreate(taskFunction, "Task", 2048, &taskIndices[i], 5, &taskControls[i].taskHandle);
        taskControls[i].lastCheckIn = xTaskGetTickCount();
    }

    // Create watchdog task
    xTaskCreate(watchdogTask, "WatchdogTask", 2048, NULL, 5, NULL);
}

void loop() {
    // Empty since we are using FreeRTOS
}
