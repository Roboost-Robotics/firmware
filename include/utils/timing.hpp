/**
 * @file timing.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Time manager for utilizing hardware timers and centralized delta time calculation.
 * @version 0.1
 * @date 2023-10-08
 * @copyright Copyright (c) 2023
 *
 * Correction focuses on ensuring that after a missed deadline the next task execution time is set properly.
 */

#ifndef TIMING_HPP
#define TIMING_HPP

#include <Arduino.h>
#include <functional>
#include <limits.h>
#include <vector>

#define TIMING_MS_TO_US(milliseconds) ((milliseconds)*1000LL)
#define TIMING_US_TO_MS(microseconds) ((microseconds) / 1000LL)
#define TIMING_S_TO_US(seconds) ((seconds)*1000000LL)
#define TIMING_US_TO_S(microseconds) ((microseconds) / 1000000LL)
#define TIMING_MS_TO_S(milliseconds) ((milliseconds) / 1000.0)
#define TIMING_S_TO_MS(seconds) ((seconds)*1000.0)
#define TIMING_MS_TO_NS(milliseconds) ((milliseconds)*1000000LL)

#define MICROS_TO_SECONDS_DOUBLE(microseconds) ((double)(microseconds) / 1000000.0)

namespace roboost
{
    namespace timing
    {

        class Task
        {
            friend class TimingService;

        public:
            Task() : callback_(nullptr), interval_(0), timeout_(0), lastScheduledRun_(0), name_(""), missed_deadlines_(0) {}

            Task(std::function<void()> callback, unsigned long interval, unsigned long timeout, const char* name)
                : callback_(callback), interval_(interval), timeout_(timeout), lastScheduledRun_(micros()), name_(name), missed_deadlines_(0)
            {
            }

            void update(unsigned long currentTime)
            {
                if (currentTime - lastScheduledRun_ >= interval_)
                {
                    unsigned long startTime = micros();
                    callback_();
                    unsigned long endTime = micros();

                    // Update lastScheduledRun_ before checking if the task execution was on time
                    if (endTime - lastScheduledRun_ > interval_)
                    {
                        lastScheduledRun_ = endTime;
                    }
                    else
                    {
                        lastScheduledRun_ += interval_;
                    }

                    // Check if task execution exceeded its timeout
                    if (endTime - startTime > timeout_)
                    {
                        // Optionally, log the timeout exceedance
                        // Serial.println("Task execution time exceeded timeout.");

                        // Increment missed deadlines safely
                        missed_deadlines_ = (missed_deadlines_ == UINT32_MAX) ? 0 : missed_deadlines_ + 1;
                    }
                }
            }

            uint32_t getMissedDeadlines() const { return missed_deadlines_; }

        private:
            std::function<void()> callback_;
            unsigned long interval_;
            unsigned long timeout_;
            unsigned long lastScheduledRun_;
            const char* name_;
            uint32_t missed_deadlines_;
        };

        class TimingService
        {
        public:
            TimingService(const TimingService&) = delete;
            TimingService& operator=(const TimingService&) = delete;

            static TimingService& get_instance()
            {
                static TimingService instance;
                return instance;
            }

            void reset() { lastUpdateTime_ = micros(); }

            void update()
            {
                unsigned long currentTime = micros();
                deltaTime_ = (currentTime < lastUpdateTime_) ? (ULONG_MAX - lastUpdateTime_ + currentTime) : (currentTime - lastUpdateTime_);
                lastUpdateTime_ = currentTime;
                updateTasks(currentTime);
            }

            void addTask(std::function<void()> callback, unsigned long interval, unsigned long timeout, const char* name) { tasks_.emplace_back(callback, interval, timeout, name); }

            void addTask(Task task) { tasks_.push_back(task); }

            unsigned long getDeltaTime() const { return deltaTime_; }

        private:
            TimingService() : lastUpdateTime_(0) { Serial.println("TimingService initialized"); }

            void updateTasks(unsigned long currentTime)
            {
                for (auto& task : tasks_)
                {
                    task.update(currentTime);
                }
            }

            unsigned long lastUpdateTime_;
            std::vector<Task> tasks_;
            unsigned long deltaTime_;
        };

        /**
         * @brief Measure the execution time of a function.
         *
         * @tparam Func The function type.
         * @tparam Args The argument types.
         * @param func The function to be measured.
         * @param args The arguments to be passed to the function.
         * @return unsigned long The execution time in microseconds.
         */
        template <typename Func, typename... Args>
        unsigned long measureExecutionTime(Func func, Args&&... args)
        {
            unsigned long startTime = micros();
            func(std::forward<Args>(args)...);
            unsigned long endTime = micros();
            return endTime - startTime;
        }

    } // namespace timing
} // namespace roboost

#endif // TIMING_HPP
