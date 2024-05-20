#include <Arduino.h>
#include <conf_hardware.h>
#include <math.h> // Include the math library for sine function
#include <roboost/motor_control/encoders/encoder.hpp>
#include <roboost/motor_control/motor_controllers/position_motor_controller.hpp>
#include <roboost/motor_control/motor_controllers/velocity_motor_controller.hpp>
#include <roboost/motor_control/motor_drivers/l298n_motor_driver.hpp>
#include <roboost/utils/controllers.hpp>
#include <roboost/utils/filters.hpp>
#include <roboost/utils/gradient_descent.hpp>
#include <roboost/utils/logging.hpp>
#include <roboost/utils/timing.hpp>
#include <roboost/utils/velocity_estimator.hpp>

using namespace roboost::motor_control;
using namespace roboost::timing;
using namespace roboost::logging;
using namespace roboost::controllers;
using namespace roboost::filters;
using namespace roboost::numeric;

SerialLogger& logger = SerialLogger::get_instance();
Scheduler& timing_service = Scheduler::get_instance(logger);

// For VelocityController
// const double kp = 1.0;
// const double ki = 0.2;
// const double kd = 0.0;

// For PositionController
const double kp = 1.0;
const double ki = 0.3;
const double kd = 0.05;

const double max_integral = 50.0;

// NoFilter derivative_filter = NoFilter();
LowPassFilter derivative_filter = LowPassFilter(10, 0.00005); // time constant = 0.001 [s]

double setpoint = 0.0;

L298NMotorDriver motor_driver(M0_IN1, M0_IN2, M0_ENA, M0_PWM_CNL);
HalfQuadEncoder encoder(M0_ENC_A, M0_ENC_B, M0_ENC_RESOLUTION, false);
PIDController controller(kp, ki, kd, max_integral, derivative_filter, &timing_service);

MovingAverageFilter encoder_filter1 = MovingAverageFilter(2);
MovingAverageFilter encoder_filter2 = MovingAverageFilter(10);
LowPassFilter encoder_filter3 = LowPassFilter(10, 0.001); // time constat = 1.0 / (2.0 * PI * 10) = 0.01591549431 [s]
LowPassFilter encoder_filter4 = LowPassFilter(10, 0.01);

// For VelocityController
const double sampling_time_derivative = 0.0000006; // 60 microseconds
const double cutoff_frequency_derivative = 15;     // 1500 Hz
// LowPassFilter input_filter = LowPassFilter(cutoff_frequency, sampling_time);
NoFilter input_filter = NoFilter();
// MovingAverageFilter input_filter = MovingAverageFilter(1000);

// For VelocityController
// NoFilter output_filter = NoFilter();

// MovingAverageFilter setpoint_filter = MovingAverageFilter(1000);
LowPassFilter setpoint_filter = LowPassFilter(1500, 0.0000006);

// For PositionController
MovingAverageFilter output_filter = MovingAverageFilter(100);

// TODO: add setpoint functions to smoothly change the setpoint
RateLimitingFilter rate_limiting_filter = RateLimitingFilter(0.00006, 0.1);
PositionController motor_controller(motor_driver, encoder, controller, input_filter, output_filter, rate_limiting_filter, 0.01, 0.05);
// VelocityController motor_controller(motor_driver, encoder, controller, input_filter, output_filter);

// Define the window size for the velocity estimator
VelocityEstimator velocityEstimator(10); // Use last 10 samples to estimate velocity

const double max_amplitude = 4 * PI; // Maximum speed in rad/s

unsigned long last_time = 0;
const double amplitude = max_amplitude;
const double frequency = 0.1;

void setup()
{
    Serial.begin(115200);
    logger.set_serial(Serial);
    encoder.set_timing_service(timing_service);
    encoder.set_logger(logger);
}

// Function for sine wave setpoint
double sineWaveSetpoint(unsigned long time) { return amplitude * sin(2 * PI * frequency * time / 1000.0); }

// Function for step response setpoint
double stepResponseSetpoint(unsigned long time)
{
    return time > 5000 ? amplitude : 0.0; // Step to amplitude after 5 seconds
}

// Function for rectangular wave setpoint
double rectangularWaveSetpoint(unsigned long time)
{
    double period = 1000.0 / frequency; // Convert frequency to period in milliseconds
    if ((time / (long)(period / 2)) % 2 == 0)
    {
        return amplitude; // Setpoint is at high amplitude
    }
    else
    {
        return 0.0; // Setpoint is reset to 0
    }
}

// Function for triangular wave setpoint
double triangularWaveSetpoint(unsigned long time)
{
    double period = 1000.0 / frequency; // Convert frequency to period in milliseconds
    double phase = time / period;
    double phase_mod = phase - (long)phase; // Get the fractional part of the phase
    if (phase_mod < 0.5)
    {
        return 4 * amplitude * phase_mod; // Setpoint is increasing
    }
    else
    {
        return 4 * amplitude * (1 - phase_mod); // Setpoint is decreasing
    }
}

double (*getSetpoint)(unsigned long) = triangularWaveSetpoint; // Function pointer to current setpoint function

void loop()
{
    timing_service.update();

    unsigned long time = TIMING_US_TO_MS(timing_service.getLastUpdateTime()); // Get time in milliseconds

    setpoint = getSetpoint(time); // Calculate setpoint based on selected function

    if (time - last_time > 100)
    {
        last_time = time;
        Serial.print(">measured[rad/s]:");
        Serial.println(encoder.get_velocity()); // TODO: Such a low resolution encoder does not work well with high update rates
        Serial.print(">measured_filtered[rad/s]:");
        Serial.println(input_filter.get_output());
        Serial.print(">setpoint[rad/s]:");
        Serial.println(setpoint);
        Serial.print(">error[rad/s]:");
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

        // delay(100);
    }

    motor_controller.update(setpoint);
}
