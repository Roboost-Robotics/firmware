/**
 * @file controllers.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Utility functions and classes for controllers.
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <Arduino.h>
#include <utils/constants.h>
#include <utils/filters.hpp>
#include <utils/timing.hpp>

namespace roboost
{
    namespace controllers
    {
        /**
         * @brief PID controller class.
         * TODO: add anti-windup, gain scheduling, and output max/min
         *
         */
        class PIDController
        {
        public:
            /**
             * @brief Construct a new PIDController object
             *
             * @param kp The proportional gain.
             * @param ki The integral gain.
             * @param kd The derivative gain.
             * @param max_expected_sampling_time The maximum expected sampling time.
             */
            PIDController(double kp, double ki, double kd, double max_expected_sampling_time, double max_integral,
                          roboost::timing::TimingService& timing_service = roboost::timing::TimingService::get_instance());

            /**
             * @brief Update the controller.
             *
             * @param setpoint The setpoint.
             * @param input The input value.
             * @return double The output value.
             */
            double update(double setpoint, double input);

            /**
             * @brief Reset the controller.
             *
             */
            void reset();

            /**
             * @brief Get the proportional gain.
             *
             * @return double The proportional gain.
             */
            double get_kp();

            /**
             * @brief Get the integral gain.
             *
             * @return double The integral gain.
             */
            double get_ki();

            /**
             * @brief Get the derivative gain.
             *
             * @return double The derivative gain.
             */
            double get_kd();

            /**
             * @brief Get the maximum expected sampling time.
             *
             * @return double The maximum expected sampling time.
             */
            double get_max_expected_sampling_time();

            /**
             * @brief Get the maximum integral.
             *
             * @return double The maximum integral.
             */
            double get_max_integral();

            /**
             * @brief Set the proportional gain.
             *
             * @param kp The proportional gain.
             */
            void set_kp(double kp);

            /**
             * @brief Set the integral gain.
             *
             * @param ki The integral gain.
             */
            void set_ki(double ki);

            /**
             * @brief Set the derivative gain.
             *
             * @param kd The derivative gain.
             */
            void set_kd(double kd);

            /**
             * @brief Set the maximum expected sampling time.
             *
             * @param max_expected_sampling_time The maximum expected sampling time.
             */
            void set_max_expected_sampling_time(double max_expected_sampling_time);

            /**
             * @brief Set the maximum integral.
             *
             * @param max_integral The maximum integral.
             */
            void set_max_integral(double max_integral);

        private:
            double kp_;
            double ki_;
            double kd_;
            double max_expected_sampling_time_;
            double max_integral_;
            double integral_;
            double previous_error_;
            roboost::filters::LowPassFilter derivative_filter_;
            roboost::timing::TimingService& timing_service_;
        };
    } // namespace controllers
} // namespace roboost

#endif // CONTROLLERS_H