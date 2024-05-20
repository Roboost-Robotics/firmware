#include <Arduino.h>
#include <conf_hardware.h>
#include <math.h>
#include <roboost/motor_control/encoders/half_quad_encoder.hpp>
#include <roboost/motor_control/motor_control_manager.hpp>
#include <roboost/motor_control/motor_controllers/position_motor_controller.hpp>
#include <roboost/motor_control/motor_drivers/motor_driver.hpp>
#include <roboost/utils/callback_scheduler.hpp>
#include <roboost/utils/controllers.hpp>
#include <roboost/utils/filters.hpp>
#include <roboost/utils/logging.hpp>
#include <roboost/utils/rtos_task_manager.hpp>
#include <roboost/utils/time_macros.hpp>

using namespace roboost::motor_control;
using namespace roboost::timing;
using namespace roboost::logging;
using namespace roboost::controllers;
using namespace roboost::filters;

Logger& logger = Logger::get_instance<SerialLogger>(Serial); // TODO: Add logger style for teleplot
CallbackScheduler& timing_service = CallbackScheduler::get_instance();
TaskManager& task_manager = TaskManager::get_instance();

// PID Controller parameters for no load (Step response)
// constexpr float kp = 0.8;
// constexpr float ki = 0;
// constexpr float kd = 0.045;

// PID Controller parameters for load (Step response)
// constexpr float kp = 4.0;
// constexpr float ki = 0;
// constexpr float kd = 0.045;

// PID Controller parameters for no load (Sine wave)
constexpr float kp = 2.8;
constexpr float ki = 0;
constexpr float kd = 0.045;

constexpr float max_integral = 50;

// Filter parameters
constexpr float cutoff_frequency_derivative = 1;
constexpr float sampling_time_derivative = 1; // Assuming time is in microseconds and needs to be scaled appropriately
constexpr uint8_t input_filter_window_size = 1;
constexpr uint8_t output_filter_window_size = 1;
constexpr float max_rate_per_second = 6; // Assuming it needs scaling
constexpr float update_rate = 1;         // Assuming time is in microseconds
constexpr float deadband_threshold = 0.01f;
constexpr float minimum_output = 0.02f;

// Motor, Encoder, and Controller instances
L298NMotorDriver motor_driver = {M3_IN1, M3_IN2, M3_ENA, M3_PWM_CNL};

HalfQuadEncoder encoder = {M3_ENC_A, M3_ENC_B, M3_ENC_RESOLUTION};

// LowPassFilter<float> derivative_filter = {cutoff_frequency_derivative, sampling_time_derivative};
NoFilter<float> derivative_filter = {};
PIDController<float> controller = {kp, ki, kd, max_integral, derivative_filter};
// MovingAverageFilter<float> input_filter = {input_filter_window_size};
NoFilter<float> input_filter = {};
// MovingAverageFilter<float> output_filter = {output_filter_window_size};
NoFilter<float> output_filter = {};
// RateLimitingFilter<float> rate_limiting_filter = {max_rate_per_second, update_rate};
NoFilter<float> rate_limiting_filter = {};

PositionController motor_controller = {motor_driver, encoder, controller, input_filter, output_filter, rate_limiting_filter, deadband_threshold, minimum_output};

const float max_amplitude = 2 * M_PI;
unsigned long last_time = 0;
const float amplitude = max_amplitude;
const float frequency = 0.1;

volatile float setpoint = 0;

// Function for sine wave setpoint
float sineWaveSetpoint(unsigned long time) { return amplitude * sin(2 * PI * frequency * time / 1000.0); }

// Function for step response setpoint
float stepResponseSetpoint(unsigned long time)
{
    return time > 5000 ? amplitude : 0; // Step to amplitude after 5 seconds
}

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

// Function for triangular wave setpoint
float triangularWaveSetpoint(unsigned long time)
{
    float period = 1000.0 / frequency; // Convert frequency to period in milliseconds
    float phase = time / period;
    float phase_mod = phase - (long)phase; // Get the fractional part of the phase
    if (phase_mod < 0.5)
    {
        return 4 * amplitude * phase_mod; // Setpoint is increasing
    }
    else
    {
        return 4 * amplitude * (1 - phase_mod); // Setpoint is decreasing
    }
}

float (*getSetpoint)(unsigned long) = sineWaveSetpoint; // Function pointer to current setpoint function

void controlLoop(void* pvParameters)
{
    while (1)
    {
        timing_service.update();

        unsigned long time = TIMING_US_TO_MS(timing_service.get_last_update_time()); // Get time in milliseconds
        setpoint = getSetpoint(time);                                                // Get setpoint from function pointer

        motor_controller.update(setpoint);

        vTaskDelay(1);
    }
}

void debugLoop(void* pvParameters)
{
    while (1)
    {
        Serial.print(">measured_vel[ticks/s]:");
        Serial.println(encoder.get_velocity());
        Serial.print(">measured_pos[ticks]:");
        Serial.println(encoder.get_position());
        Serial.print(">measured_pos[rad]:");
        Serial.println(encoder.ticks_to_radians(encoder.get_position()));
        Serial.print(">setpoint[rad]:");
        Serial.println(setpoint);
        Serial.print(">error[rad]:");
        Serial.println(setpoint - input_filter.get_output());
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
        Serial.print(">position_setpoint:");
        Serial.println(motor_controller.get_setpoint());

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
        // motor_driver.set_motor_control(1 << 10); // Set motor to full speed
        delay(1000);
    }
}

void loop() {}
