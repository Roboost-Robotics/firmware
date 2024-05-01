#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

void task1Function(void *parameters) {
    const char* message = static_cast<const char*>(parameters);
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS; // 0.5 second delay

    // Register task with watchdog timer
    esp_task_wdt_add(NULL);

    while (true) {
        Serial.println(message);
        vTaskDelay(xDelay); // Delay for 1 second

        // Reset the watchdog timer
        esp_task_wdt_reset();
    }
}

void task2Function(void *parameters) {
    const char* message = static_cast<const char*>(parameters);
    const TickType_t xDelay = 1000 / portTICK_PERIOD_MS; // 1 second delay

    // Register task with watchdog timer
    esp_task_wdt_add(NULL);

    while (true) {
        Serial.println(message);
        vTaskDelay(xDelay); // Delay for 1 second

        // Reset the watchdog timer
        esp_task_wdt_reset();
    }
}// Task function for a periodic task with fixed execution time
void periodicTask(void *pvParameters) {
    const TickType_t xPeriod = pdMS_TO_TICKS(1000); // Task period of 1 second
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // Record start time
        TickType_t xStartTime = xTaskGetTickCount();

        // Perform task operations
        // Example: Do something that takes time
        vTaskDelay(pdMS_TO_TICKS(200)); // Simulated task execution time of 200ms

        // Record end time and calculate execution time
        TickType_t xEndTime = xTaskGetTickCount();
        TickType_t xExecutionTime = xEndTime - xStartTime;

        // Check if execution time exceeds threshold
        const TickType_t xMaxExecutionTime = pdMS_TO_TICKS(50); // Maximum allowed execution time of 50ms
        if (xExecutionTime > xMaxExecutionTime) {
            // Handle the case where execution time exceeds the threshold
            // Example: Log an error message or take corrective action
            Serial.println("Task execution time exceeded!");
        }

        // Wait until next period
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void setup() {
    Serial.begin(115200);

    // Create tasks
    // xTaskCreate(task1Function, "Task 1", 2048, (void*)"Task runs every 0.5 second", 5, NULL);
    // xTaskCreate(task2Function, "Task 2", 2048, (void*)"Task runs every 1 seconds", 5, NULL);
    xTaskCreate(periodicTask, "PeriodicTask", 2048, NULL, 1, NULL); // Adjust stack size and priority
}

void loop() {
    // In FreeRTOS, the loop function should not contain any code and should return.
}
