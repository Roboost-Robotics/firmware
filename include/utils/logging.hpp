/**
 * @file logging.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Utility functions for logging.
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

namespace roboost
{
    namespace logging
    {

        // Abstract Logger interface
        class Logger
        {
        public:
            virtual void info(const String& message) = 0;
            virtual void warn(const String& message) = 0;
            virtual void error(const String& message) = 0;
            virtual void debug(const String& message) = 0;
            virtual ~Logger() {}
        };

        // SerialLogger singleton
        class SerialLogger : public Logger
        {
        private:
            usb_serial_class& serial;

            // Private constructor
            SerialLogger(usb_serial_class& serial) : serial(serial) {}

            // Prevent copying and assignment
            SerialLogger(const SerialLogger&) = delete;
            SerialLogger& operator=(const SerialLogger&) = delete;

        public:
            static SerialLogger& getInstance(usb_serial_class& serial)
            {
                static SerialLogger instance(serial);
                return instance;
            }

            void info(const String& message) override
            {
                serial.print("[INFO] ");
                serial.println(message);
            }

            void warn(const String& message) override
            {
                serial.print("[WARN] ");
                serial.println(message);
            }

            void error(const String& message) override
            {
                serial.print("[ERROR] ");
                serial.println(message);
            }

            void debug(const String& message) override
            {
                serial.print("[DEBUG] ");
                serial.println(message);
            }
        };

    }; // namespace logging
};     // namespace roboost

#endif // LOGGER_H
