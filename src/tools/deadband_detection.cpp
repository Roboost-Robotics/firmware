#include <Arduino.h>
#include <conf_hardware.h>
#include <math.h> // Include the math library for sine function
#include <roboost/motor_control/encoders/encoder.hpp>
#include <roboost/motor_control/motor_controllers/position_motor_controller.hpp>
#include <roboost/motor_control/motor_controllers/velocity_motor_controller.hpp>
#include <roboost/motor_control/motor_drivers/l298n_motor_driver.hpp>
#include <roboost/utils/controllers.hpp>
#include <roboost/utils/filters.hpp>
#include <roboost/utils/logging.hpp>
#include <roboost/utils/regression.hpp>
#include <roboost/utils/timing.hpp>

using namespace roboost::motor_control;
using namespace roboost::timing;
using namespace roboost::logging;
using namespace roboost::controllers;
using namespace roboost::filters;

L298NMotorDriver motor_driver(M3_IN1, M3_IN2, M3_ENA, M3_PWM_CNL);
HalfQuadEncoder encoder(M3_ENC_A, M3_ENC_B, M3_ENC_RESOLUTION, false);
SerialLogger& logger = SerialLogger::get_instance();
Scheduler& timing_service = Scheduler::get_instance();

void setup()
{
    Serial.begin(115200);

    encoder.set_timing_service(timing_service);
    encoder.set_logger(logger);

    // Assuming the motor driver setup includes configuration for PWM, directions, etc.
    motor_driver.set_motor_control(0); // Ensure motor is stopped initially
    delay(2000);                       // Wait for system to stabilize

    Serial.println("Starting deadband and minimum output test...");

    double output = 0.0;
    double increment = 0.001; // Increment size for control output
    bool movementDetected = false;
    double firstMovementOutput = 0.0;

    while (!movementDetected && output <= 1.0)
    {
        motor_driver.set_motor_control(output);
        delay(10); // Delay to allow motor response and encoder readings

        encoder.update();                      // Update encoder to get the latest velocity
        double speed = encoder.get_velocity(); // Current motor speed

        if (speed > 0.001)
        {
            movementDetected = true;
            firstMovementOutput = output;
            Serial.print("Movement detected at output: ");
            Serial.println(output, DEC);
        }
        else
        {
            Serial.print("No movement at output: ");
            Serial.println(output, DEC);
        }

        output += increment; // Increase the control output
    }

    if (!movementDetected)
    {
        Serial.println("No movement detected within the test range.");
    }

    motor_driver.set_motor_control(0); // Stop the motor
}

void loop()
{
    delay(1000); // Loop delay
}