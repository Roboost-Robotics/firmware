#include <Arduino.h>
#include <conf_hardware.h>
#include <math.h>
#include <roboost/motor_control/encoders/half_quad_encoder.hpp>
#include <roboost/motor_control/motor_control_manager.hpp>
#include <roboost/motor_control/motor_controllers/position_motor_controller.hpp>
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

Logger& logger = Logger::get_instance<SerialLogger>(Serial); // TODO: Add logger style for teleplot
CallbackScheduler& timing_service = CallbackScheduler::get_instance();
TaskManager& task_manager = TaskManager::get_instance();

constexpr size_t OUTPUT_VALUES_SIZE = 50;
// Save the last OUTPUT_VALUES_SIZE values of the output
std::vector<float> output_values(OUTPUT_VALUES_SIZE);

// PID Controller parameters for no load (Sine wave)
constexpr float kp = 2000.8; // 2.8
constexpr float ki = 0.0;    // 0
constexpr float kd = 0.045;  // 0.045

constexpr float max_integral = 1000;

// Filter parameters
constexpr float cutoff_frequency_derivative = 20;
constexpr float sampling_time_derivative = 0.001; // Assuming time is in microseconds and needs to be scaled appropriately
constexpr uint8_t input_filter_window_size = 80;
constexpr uint8_t output_filter_window_size = 1;
constexpr float max_rate_per_second = 0.8; // Assuming it needs scaling
constexpr float update_rate = 0.000001;    // Assuming time is in microseconds
constexpr float deadband_threshold = 0.01f;
constexpr float minimum_output = 0.02f;

// Motor, Encoder, and Controller instances arrays
L298NMotorDriver motor_drivers[4] = {{M0_IN1, M0_IN2, M0_ENA, M0_PWM_CNL}, {M1_IN1, M1_IN2, M1_ENA, M1_PWM_CNL}, {M2_IN1, M2_IN2, M2_ENA, M2_PWM_CNL}, {M3_IN1, M3_IN2, M3_ENA, M3_PWM_CNL}};

HalfQuadEncoder encoders[4] = {{M0_ENC_A, M0_ENC_B, M0_ENC_RESOLUTION}, {M1_ENC_A, M1_ENC_B, M1_ENC_RESOLUTION}, {M2_ENC_A, M2_ENC_B, M2_ENC_RESOLUTION}, {M3_ENC_A, M3_ENC_B, M3_ENC_RESOLUTION}};

LowPassFilter<float> derivative_filter = {cutoff_frequency_derivative, sampling_time_derivative};
PIDController<float> controllers[4] = {
    {kp, ki, kd, max_integral, derivative_filter}, {kp, ki, kd, max_integral, derivative_filter}, {kp, ki, kd, max_integral, derivative_filter}, {kp, ki, kd, max_integral, derivative_filter}};

NoFilter<float> input_filter = {};
NoFilter<float> output_filter = {};
NoFilter<float> rate_limiting_filter = {};

PositionController motor_controllers[4] = {{motor_drivers[0], encoders[0], controllers[0], input_filter, output_filter, rate_limiting_filter, deadband_threshold, minimum_output},
                                           {motor_drivers[1], encoders[1], controllers[1], input_filter, output_filter, rate_limiting_filter, deadband_threshold, minimum_output},
                                           {motor_drivers[2], encoders[2], controllers[2], input_filter, output_filter, rate_limiting_filter, deadband_threshold, minimum_output},
                                           {motor_drivers[3], encoders[3], controllers[3], input_filter, output_filter, rate_limiting_filter, deadband_threshold, minimum_output}};

IncrementalEncoderVelocityEstimator velocity_estimators[4] = {{0.000001, 0.01, 50}, {0.000001, 0.01, 50}, {0.000001, 0.01, 50}, {0.000001, 0.01, 50}};

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

float (*getSetpoint)(unsigned long) = triangularWaveSetpoint; // Function pointer to current setpoint function

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

        for (int i = 0; i < 4; ++i)
        {
            motor_controllers[i].update(setpoint);
        }

        // Put the current output in the output_values vector
        output_values.push_back(controllers[2].get_output());
        if (output_values.size() > OUTPUT_VALUES_SIZE)
        {
            output_values.erase(output_values.begin());
        }

        // Wait for the next cycle.
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void debugLoop(void* pvParameters)
{
    while (1)
    {
        Serial.print(">estimated_vel[rad/s]:");
        Serial.println(velocity_estimators[2].get_output());
        Serial.print(">measured_pos[ticks]:");
        Serial.println(encoders[2].get_position());
        Serial.print(">measured_pos[rad]:");
        Serial.println(encoders[2].get_position_radians());
        Serial.print(">setpoint[rad]:");
        Serial.println(setpoint);
        Serial.print(">error[rad]:");
        Serial.println(setpoint - input_filter.get_output());
        Serial.print(">dt[us]:");
        Serial.println(timing_service.get_delta_time());
        Serial.print(">P:");
        Serial.println(controllers[2].get_previous_error() * kp);
        Serial.print(">I:");
        Serial.println(controllers[2].get_integral() * ki);
        Serial.print(">D:");
        Serial.println(controllers[2].get_derivative() * kd);
        Serial.print(">output:");
        Serial.println(controllers[2].get_output());
        Serial.print(">filtered_output:");
        Serial.println(output_filter.get_output());
        Serial.print(">control_value:");
        Serial.println(motor_drivers[2].get_motor_control());
        Serial.print(">position_setpoint:");
        Serial.println(motor_controllers[2].get_setpoint());
        for (auto value : output_values)
        {
            Serial.print(">output_values:");
            Serial.println(value);
        }

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
