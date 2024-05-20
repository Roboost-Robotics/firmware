#include <Arduino.h>
#include <conf_hardware.h>
#include <math.h>
#include <roboost/kinematics/kinematics.hpp>
#include <roboost/motor_control/encoders/encoder.hpp>
#include <roboost/motor_control/motor_control_manager.hpp>
#include <roboost/motor_control/motor_controllers/velocity_motor_controller.hpp>
#include <roboost/motor_control/motor_drivers/motor_driver.hpp>
#include <roboost/motor_control/robot_controller.hpp>
#include <roboost/utils/callback_scheduler.hpp>
#include <roboost/utils/controllers.hpp>
#include <roboost/utils/filters.hpp>
#include <roboost/utils/logging.hpp>
#include <roboost/utils/matrices.hpp>
#include <roboost/utils/rtos_task_manager.hpp>
#include <roboost/utils/time_macros.hpp>

#define MOTOR_COUNT 4

using namespace roboost::motor_control;
using namespace roboost::timing;
using namespace roboost::logging;
using namespace roboost::controllers;
using namespace roboost::filters;
using namespace roboost::math;
using namespace roboost::kinematics;
using namespace roboost::robot_controller;

Logger& logger = Logger::get_instance<SerialLogger>(Serial);
CallbackScheduler& timing_service = CallbackScheduler::get_instance();
TaskManager& task_manager = TaskManager::get_instance();

// PID Controller parameters
constexpr float kp = 1.0;
constexpr float ki = 0;
constexpr float kd = 0;
constexpr float max_integral = 50;

// Filter parameters
constexpr float cutoff_frequency_derivative = 100000;
constexpr float sampling_time_derivative = 1; // Assuming time is in microseconds and needs to be scaled appropriately
constexpr float cutoff_frequency_setpoint = 150;
constexpr float sampling_time_setpoint = 1; // Assuming time is in microseconds and needs to be scaled appropriately
constexpr uint8_t output_filter_window_size = 100;
constexpr uint8_t input_filter_window_size = 100;
constexpr float max_rate_per_second = 6;   // Assuming it needs scaling
constexpr float update_rate = 10000;       // Assuming time is in microseconds
constexpr int32_t deadband_threshold = 10; // example value, in ticks/s
constexpr int32_t minimum_output = 50;     // example value, scaled by 1024

// Motor, Encoder, and Controller instances
// clang-format off
L298NMotorDriver motor_driver[MOTOR_COUNT] = {
    {M0_IN1, M0_IN2, M0_ENA, M0_PWM_CNL},
    {M1_IN1, M1_IN2, M1_ENA, M1_PWM_CNL},
    {M2_IN1, M2_IN2, M2_ENA, M2_PWM_CNL},
    {M3_IN1, M3_IN2, M3_ENA, M3_PWM_CNL}};

#ifdef ESP32
HalfQuadEncoder encoder[MOTOR_COUNT] = {
    {M0_ENC_A, M0_ENC_B, M0_ENC_RESOLUTION},
    {M1_ENC_A, M1_ENC_B, M1_ENC_RESOLUTION},
    {M2_ENC_A, M2_ENC_B, M2_ENC_RESOLUTION},
    {M3_ENC_A, M3_ENC_B, M3_ENC_RESOLUTION}};
#else
DummyEncoder encoder[MOTOR_COUNT] = {
    {M0_ENC_RESOLUTION},
    {M1_ENC_RESOLUTION},
    {M2_ENC_RESOLUTION},
    {M3_ENC_RESOLUTION}};
#endif

LowPassFilter<float> derivative_filter[MOTOR_COUNT] = {
    {cutoff_frequency_derivative, sampling_time_derivative},
    {cutoff_frequency_derivative, sampling_time_derivative},
    {cutoff_frequency_derivative, sampling_time_derivative},
    {cutoff_frequency_derivative, sampling_time_derivative}};

PIDController<float, LowPassFilter<float>> controller[MOTOR_COUNT] = {
    {kp, ki, kd, max_integral, derivative_filter[0]},
    {kp, ki, kd, max_integral, derivative_filter[1]},
    {kp, ki, kd, max_integral, derivative_filter[2]},
    {kp, ki, kd, max_integral, derivative_filter[3]}};

MovingAverageFilter<float> input_filter[MOTOR_COUNT] = {
    {input_filter_window_size},
    {input_filter_window_size},
    {input_filter_window_size},
    {input_filter_window_size}};

MovingAverageFilter<float> output_filter[MOTOR_COUNT] = {
    {output_filter_window_size},
    {output_filter_window_size},
    {output_filter_window_size},
    {output_filter_window_size}};

RateLimitingFilter<float> rate_limiting_filter[MOTOR_COUNT] = {
    {max_rate_per_second, update_rate},
    {max_rate_per_second, update_rate},
    {max_rate_per_second, update_rate},
    {max_rate_per_second, update_rate}};

VelocityController<L298NMotorDriver,
                    HalfQuadEncoder,
                    PIDController<float, LowPassFilter<float>>,
                    MovingAverageFilter<float>,
                    MovingAverageFilter<float>,
                    RateLimitingFilter<float>>
    motor_controller[MOTOR_COUNT] = {
        {motor_driver[0], encoder[0], controller[0], input_filter[0], output_filter[0], rate_limiting_filter[0], deadband_threshold, minimum_output},
        {motor_driver[1], encoder[1], controller[1], input_filter[1], output_filter[1], rate_limiting_filter[1], deadband_threshold, minimum_output},
        {motor_driver[2], encoder[2], controller[2], input_filter[2], output_filter[2], rate_limiting_filter[2], deadband_threshold, minimum_output},
        {motor_driver[3], encoder[3], controller[3], input_filter[3], output_filter[3], rate_limiting_filter[3], deadband_threshold, minimum_output}};

MotorControllerManager<
    VelocityController<L298NMotorDriver,
                        HalfQuadEncoder,
                        PIDController<float, LowPassFilter<float>>,
                        MovingAverageFilter<float>,
                        MovingAverageFilter<float>,
                        RateLimitingFilter<float>>>
    motor_manager = {
        &motor_controller[0],
        &motor_controller[1],
        &motor_controller[2],
        &motor_controller[3]};

MecanumKinematics4W kinematics_model(50, 300, 300); // Units in mm

RobotVelocityController<
    VelocityController<L298NMotorDriver,
                        HalfQuadEncoder,
                        PIDController<float, LowPassFilter<float>>,
                        MovingAverageFilter<float>,
                        MovingAverageFilter<float>,
                        RateLimitingFilter<float>>,
                        MecanumKinematics4W>
    robot_controller(motor_manager, kinematics_model);
// clang-format on

const float max_amplitude = 2 * M_PI;
unsigned long last_time = 0;
const float amplitude = max_amplitude;
const float frequency = 0.1;

volatile int32_t setpoint = 0;

// Function for sine wave setpoint
int32_t sineWaveSetpoint(unsigned long time) { return amplitude * sin(2 * PI * frequency * time / 1000.0); }

// Function for step response setpoint
int32_t stepResponseSetpoint(unsigned long time)
{
    return time > 5000 ? amplitude : 0; // Step to amplitude after 5 seconds
}

// Function for rectangular wave setpoint
int32_t rectangularWaveSetpoint(unsigned long time)
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
int32_t triangularWaveSetpoint(unsigned long time)
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

int32_t (*getSetpoint)(unsigned long) = stepResponseSetpoint; // Function pointer to current setpoint function

void controlLoop(void* pvParameters)
{
    while (1)
    {
        timing_service.update();

        unsigned long time = TIMING_US_TO_MS(timing_service.get_last_update_time()); // Get time in milliseconds
        setpoint = getSetpoint(time);                                                // Get setpoint from function pointer

        Vector<float> target_velocity(3);
        target_velocity[0] = setpoint; // vx
        target_velocity[1] = 0;        // vy
        target_velocity[2] = 0;        // vz

        robot_controller.set_latest_command(target_velocity);
        robot_controller.update();

        vTaskDelay(1);
    }
}

void debugLoop(void* pvParameters)
{
    while (1)
    {
        Serial.print(">control_task_exec_time[us]:");
        Serial.println(timing_service.get_delta_time());
        Serial.print(">debug_task_priority:");

        Serial.print(">measured_vel[ticks/s]:");
        Serial.println(encoder[3].get_velocity()); // TODO: Such a low resolution encoder does not work well with high update rates
        Serial.print(">measured_vel_filtered[ticks/s]:");
        Serial.println(input_filter[3].get_output());
        Serial.print(">measured_pos[ticks]:");
        Serial.println(encoder[3].get_position());
        Serial.print(">setpoint[ticks/s]:");
        Serial.println(setpoint);
        Serial.print(">error[ticks/s]:");
        Serial.println(setpoint - input_filter[3].get_output());
        Serial.print(">dt[us]:");
        Serial.println(timing_service.get_delta_time());
        Serial.print(">P:");
        Serial.println(controller[3].get_previous_error() * kp / 1024);
        Serial.print(">I:");
        Serial.println(controller[3].get_integral() * ki / 1024);
        Serial.print(">D:");
        Serial.println(controller[3].get_derivative() * kd / 1024);
        Serial.print(">control_value:");
        Serial.println(motor_driver[3].get_motor_control());
        Serial.print(">position_setpoint:");
        Serial.println(motor_controller[3].get_setpoint());

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup()
{
    Serial.begin(115200);

    Serial.print(">start:");
    Serial.println(millis());

    task_manager.create_task(controlLoop, "ControlTask", 2048, NULL, (configMAX_PRIORITIES - 1));
    task_manager.create_task(debugLoop, "DebugTask", 2048, NULL, 1);

    // Disable loop
    while (1)
    {
        // motor_driver[3].set_motor_control(1 << 10); // Set motor to full speed
        // delay(1000);
    }
}

void loop() {}
