#include <Arduino.h>
#include <vector>
#include <functional>

struct Task {
    uint32_t interval; // Interval in microseconds
    uint64_t next; // Next scheduled execution time in microseconds
    std::function<void()> callback; // Function to execute
    uint32_t executionThreshold; // Maximum allowed execution time in microseconds
};

class Scheduler {
private:
    std::vector<Task> tasks;

public:
    void addTask(uint32_t intervalMs, uint32_t execThresholdMs, std::function<void()> callback) {
        Task newTask;
        newTask.interval = intervalMs * 1000;
        newTask.next = micros() + newTask.interval;
        newTask.callback = callback;
        newTask.executionThreshold = execThresholdMs * 1000;
        tasks.push_back(newTask);
    }

    void run() {
        uint64_t now = micros();
        for (auto &task : tasks) {
            if (now >= task.next) {
                uint64_t startExecution = micros();
                task.callback();
                uint64_t executionTime = micros() - startExecution;
                if (executionTime > task.executionThreshold) {
                    // TODO: Handle this case
                    Serial.println("Execution time exceeded!");
                }
                task.next += task.interval;
                if (task.next < now) {
                    task.next = now + task.interval;
                }
            }
        }
    }
};

Scheduler scheduler;

void setup() {
    Serial.begin(115200);
    scheduler.addTask(1000, 950, []() { Serial.println("Task runs every 1 second"); }); // delay(1000); }); // Delay to simulate a long running task
    scheduler.addTask(500, 450, []() { Serial.println("Task runs every 0.5 second"); });
}

void loop() {
    scheduler.run();
    delay(1);
}
