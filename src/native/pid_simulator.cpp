#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <matplotlibcpp.h>
#include <roboost/utils/controllers.hpp>
#include <roboost/utils/filters.hpp>
#include <roboost/utils/timing.hpp>
#include <vector>

using namespace roboost::controllers;
using namespace roboost::timing;
using namespace roboost::filters;
using namespace roboost::logging;

namespace plt = matplotlibcpp;

// Function to generate a sine wave setpoint
std::function<double(double)> createSineWaveFunction(double amplitude, double frequency)
{
    return [amplitude, frequency](double time) -> double { return amplitude * sin(2 * M_PI * frequency * time); };
}

// Function to generate a step wave setpoint
std::function<double(double)> createStepFunction(double value, double startTime)
{
    return [value, startTime](double time) -> double { return time >= startTime ? value : 0.0; };
}

// Function to generate a ramp wave setpoint
std::function<double(double)> createRampFunction(double slope, double startTime)
{
    return [slope, startTime](double time) -> double { return time >= startTime ? slope * (time - startTime) : 0.0; };
}

// Function to simulate different system dynamics
double simulateSystem(double input, double& output, double dt, const std::vector<double>& timeConstants)
{
    double newOutput = output;
    for (double tau : timeConstants)
    {
        newOutput += (input - newOutput) * dt / tau;
    }
    return newOutput;
}

int main()
{
    // Simulation and system parameters
    const double sim_time = 1.0; // Simulation time in seconds
    const double dt = 0.01;      // Time step in seconds
    const int num_steps = static_cast<int>(sim_time / dt);

    // Setpoint function
    // auto setpointFunction = createSineWaveFunction(1.0, 1.0); // Sine wave with amplitude 1.0 and frequency 1.0 Hz
    auto setpointFunction = createStepFunction(1.0, 0.1); // Step function with value 1.0 starting at 0.5 seconds
    // auto setpointFunction = createRampFunction(1.0, 0.5); // Ramp function with slope 1.0 starting at 0.5 seconds

    // System characteristics
    std::vector<double> timeConstants = {0.01}; // Can add multiple time constants for more complex dynamics

    // PID controller setup
    float kp = 0.6, ki = 1.05, kd = 0.001;
    LowPassFilter derivative_filter(1.0 / (1.0 + 2.0 * M_PI * kd * dt), dt);

    ConsoleLogger& logger = ConsoleLogger::getInstance();
    Scheduler timing_service = Scheduler::get_instance(logger);
    timing_service.setDeltaTime(TIMING_S_TO_US(dt));

    PIDControllerConfig config{kp, ki, kd, 10.0, derivative_filter, &timing_service};
    PIDController pid(config);

    std::ofstream dataFile("sim/pid_output.csv");
    dataFile << "Time,Setpoint,System Output,Control Value,Error,Integral,Derivative\n";

    double system_output = 0.0;
    double time = 0.0;

    // Vectors to store data for plotting
    std::vector<double> times, setpoints, outputs, control_values;

    for (int i = 0; i < num_steps; ++i)
    {
        time = i * dt;
        double setpoint = setpointFunction(time);
        double control_value = pid.update(setpoint, system_output);
        system_output = simulateSystem(control_value, system_output, dt, timeConstants);

        // Store data for plotting
        times.push_back(time);
        setpoints.push_back(setpoint);
        outputs.push_back(system_output);
        control_values.push_back(control_value);
    }

    // Plotting
    plt::figure();
    plt::named_plot("Setpoint", times, setpoints);
    plt::named_plot("System Output", times, outputs);
    plt::named_plot("Control Value", times, control_values);
    plt::xlabel("Time (s)");
    plt::ylabel("Value");
    plt::title("PID Controller Simulation");
    plt::legend();
    plt::show();

    std::cout << "Simulation and plotting completed." << std::endl;
    return 0;
}
