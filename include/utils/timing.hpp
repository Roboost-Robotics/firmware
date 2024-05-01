/**
 * @file timing.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Time manager for utilizing hardware timers and centralized delta time calculation.
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef TIMING_HPP
#define TIMING_HPP

#include <Arduino.h>
#include <vector>
#include <functional>

#define TIMING_MS_TO_US(milliseconds) ((milliseconds) * 1000LL)
#define TIMING_US_TO_MS(microseconds) ((microseconds) / 1000LL)
#define TIMING_S_TO_US(seconds) ((seconds) * 1000000LL)
#define TIMING_US_TO_S(microseconds) ((microseconds) / 1000000LL)

#define MICROS_TO_SECONDS_DOUBLE(microseconds) ((double)(microseconds) / 1000000.0)

class Task {
public:
    Task(std::function<void()> callback, unsigned long interval, unsigned long timeout, const char* name)
    : callback_(callback), interval_(interval), timeout_(timeout), lastScheduledRun_(micros()), name_(name) {
        // Serial.print("Task ");
        // Serial.print(name_);
        // Serial.print(" initialized with interval ");
        // Serial.print(interval_);
        // Serial.print(" us and timeout ");
        // Serial.println(timeout_);
        // Serial.println(" us.");
    }

    void update(unsigned long currentTime) {
        if (currentTime - lastScheduledRun_ >= interval_) {
            unsigned long startTime = micros();
            callback_();
            unsigned long endTime = micros();

            // Check if task execution was on time
            if (endTime - lastScheduledRun_ > timeout_) {
                Serial.print("Warning: Task \"");
                Serial.print(name_);
                Serial.print("\" has exceeded its timeout period. Execution took ");
                Serial.print(endTime - startTime);
                Serial.println(" us.");

            }

            if (endTime - lastScheduledRun_ > interval_) {
                lastScheduledRun_ = endTime; // Reschedule from the end time if delayed
            } else {
                lastScheduledRun_ += interval_; // Otherwise, keep consistent intervals
            }

        }
    }

private:
    std::function<void()> callback_;
    unsigned long interval_;        // Expected interval for task execution
    unsigned long timeout_;         // Maximum allowed delay before considering it a timeout
    unsigned long lastScheduledRun_; // Last scheduled run time (to manage exact interval scheduling)
    const char* name_;
};



// Singleton class for managing timing
class TimingService {
public:
    // Delete the copy constructor and copy assignment operator
    TimingService(const TimingService&) = delete;
    TimingService& operator=(const TimingService&) = delete;

    // Provide a static method to access the instance
    static TimingService& get_instance() {
        static TimingService instance;
        return instance;
    }

    void reset() {
        lastUpdateTime_ = micros();
    }

    unsigned long update() {
        unsigned long currentTime = micros();
        if (currentTime < lastUpdateTime_) {
            // Handle wrap-around of micros()
            deltaTime_ = (ULONG_MAX - lastUpdateTime_ + currentTime);
        } else {
            deltaTime_ = (currentTime - lastUpdateTime_);
        }
        lastUpdateTime_ = currentTime;
        updateTasks(currentTime);
        return deltaTime_;
    }

    void addTask(std::function<void()> callback, unsigned long interval, unsigned long timeout, char* name) {
        tasks_.emplace_back(callback, interval, timeout, name);
    }

    unsigned long getDeltaTime() {
        return deltaTime_;
    }

private:
    TimingService() : lastUpdateTime_(0), deltaTime_(0) {

        Serial.println("TimingService initialized");
    }

    void updateTasks(unsigned long currentTime) {
        for (auto& task : tasks_) {
            task.update(currentTime);
        }
    }

    unsigned long lastUpdateTime_;
    unsigned long deltaTime_;
    std::vector<Task> tasks_;
};

#endif // TIMING_HPP
