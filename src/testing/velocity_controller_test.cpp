#include <Arduino.h>
#include <conf_hardware.h>
#include <math.h>
#include <roboost/motor_control/encoders/half_quad_encoder.hpp>
#include <roboost/motor_control/motor_control_manager.hpp>
#include <roboost/motor_control/motor_controllers/position_motor_controller.hpp>
#include <roboost/motor_control/motor_controllers/velocity_motor_controller.hpp>
#include <roboost/motor_control/motor_drivers/motor_driver.hpp>
#include <roboost/utils/callback_scheduler.hpp>
#include <roboost/utils/controllers.hpp>
#include <roboost/utils/estimators.hpp>
#include <roboost/utils/filters.hpp>
#include <roboost/utils/logging.hpp>
#include <roboost/utils/rtos_task_manager.hpp>
#include <roboost/utils/time_macros.hpp>

using namespace roboost::motor_control;
using namespace roboost::timing;
using namespace roboost::logging;
using namespace roboost::controllers;
using namespace roboost::filters;
using namespace roboost::estimators;

// Logger and scheduler instances
Logger& logger = Logger::get_instance<SerialLogger>(Serial);
CallbackScheduler& timing_service = CallbackScheduler::get_instance();
TaskManager& task_manager = TaskManager::get_instance();

// PID Controller parameters for no load (Sine wave)
constexpr float kp = 2;     // 2.8
constexpr float ki = 0.0;   // 0
constexpr float kd = 0.045; // 0.045
constexpr float max_integral = 4000;

// Motor, Encoder, and Controller instances
L298NMotorDriver motor_driver = {M3_IN1, M3_IN2, M3_ENA, M3_PWM_CNL};
HalfQuadEncoder encoder = {M3_ENC_A, M3_ENC_B, M3_ENC_RESOLUTION};

NoFilter<float> derivative_filter = {};
NoFilter<float> input_filter = {};
NoFilter<float> output_filter = {};

NoEstimator velocity_estimator = {};

// PID Controller for velocity control
PIDController<float> controller = {kp, ki, kd, max_integral, derivative_filter};
VelocityController_V2 motor_controller = {motor_driver, encoder, controller, input_filter, output_filter, derivative_filter, 0.01, 0.02, true};

// Setpoint parameters
const float max_amplitude = 2 * M_PI;
const float amplitude = max_amplitude;
const float frequency = 0.02;
volatile float setpoint = 0;

// Function for rectangular wave setpoint
float rectangularWaveSetpoint(unsigned long time)
{
    float period = 1000.0 / frequency; // Convert frequency to period in milliseconds
    if ((time / (long)(period / 2)) % 2 == 0)
    {
        return amplitude; // Setpoint is at high amplitude
    }
    else
    {
        return 0; // Setpoint is reset to 0
    }
}

float (*getSetpoint)(unsigned long) = rectangularWaveSetpoint; // Function pointer to current setpoint function

void controlLoop(void* pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1 tick

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        timing_service.update();

        unsigned long time = TIMING_US_TO_MS(timing_service.get_last_update_time()); // Get time in milliseconds
        setpoint = getSetpoint(time);                                                // Get setpoint from function pointer

        motor_controller.update(setpoint);

        // Serial.print(">measured_pos[ticks]:");
        // Serial.println(encoder.get_position());
        // Serial.print(">filtered_vel[rad/s]:");
        // Serial.println(input_filter.get_output());
        // Serial.print(">setpoint[rad/s]:");
        // Serial.println(setpoint);
        // Serial.print(">error[rad]:");
        // Serial.println(controller.get_previous_error());
        // Serial.print(">dt[us]:");
        // Serial.println(timing_service.get_delta_time());
        // Serial.print(">P:");
        // Serial.println(controller.get_previous_error() * kp);
        // Serial.print(">I:");
        // Serial.println(controller.get_integral() * ki);
        // Serial.print(">D:");
        // Serial.println(controller.get_derivative() * kd);
        // Serial.print(">control_value:");
        // Serial.println(motor_driver.get_motor_control());
        // Serial.print(">filtered_output:");
        // Serial.println(output_filter.get_output());
        // Serial.print(">position_setpoint:");
        // Serial.println(motor_controller.get_setpoint());
        // Serial.print(">estimated_vel[rad/s]:");
        // Serial.println(velocity_estimator.get_output());
        // Serial.print(">delta_time[s]:");
        // Serial.println(velocity_estimator.get_delta_time());
        // Serial.print(">delta_position[rad]:");
        // Serial.println(velocity_estimator.get_delta_position());

        // Wait for the next cycle.
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void debugLoop(void* pvParameters)
{
    while (1)
    {
        Serial.print(">measured_pos[ticks]:");
        Serial.println(encoder.get_position());
        Serial.print(">filtered_vel[rad/s]:");
        Serial.println(input_filter.get_output());
        Serial.print(">setpoint[rad/s]:");
        Serial.println(setpoint);
        Serial.print(">error[rad]:");
        Serial.println(controller.get_previous_error());
        Serial.print(">dt[us]:");
        Serial.println(timing_service.get_delta_time());
        Serial.print(">P:");
        Serial.println(controller.get_previous_error() * kp);
        Serial.print(">I:");
        Serial.println(controller.get_integral() * ki);
        Serial.print(">D:");
        Serial.println(controller.get_derivative() * kd);
        Serial.print(">control_value:");
        Serial.println(motor_driver.get_motor_control());
        Serial.print(">filtered_output:");
        Serial.println(output_filter.get_output());
        Serial.print(">position_setpoint:");
        Serial.println(motor_controller.get_setpoint());
        Serial.print(">estimated_vel[rad/s]:");
        Serial.println(velocity_estimator.get_output());
        Serial.print(">timing[s]:");
        Serial.println(TIMING_US_TO_S_DOUBLE(timing_service.get_delta_time()) * 1000);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup()
{
    Serial.begin(115200);

    task_manager.create_task(controlLoop, "ControlTask", 2048, NULL, (configMAX_PRIORITIES - 1));
    task_manager.create_task(debugLoop, "DebugTask", 2048, NULL, 1);

    // Disable loop
    while (1)
    {
        delay(1000);
    }
}

void loop() {}
