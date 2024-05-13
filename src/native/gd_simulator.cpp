#include <cmath>
#include <functional>
#include <iostream>
#include <matplotlibcpp.h>
#include <roboost/utils/gradient_descent.hpp>
#include <vector>

namespace plt = matplotlibcpp;

// Function to calculate the cost
double costFunction(const std::vector<double>& params)
{
    return std::pow(params[0] - 3, 2); // Cost function (x-3)^2
}

int main()
{
    // Initial settings
    std::vector<double> initial_params = {10}; // Starting from x = 10
    double learning_rate = 0.1;                // A suitable learning rate
    int iterations = 100;                      // Number of iterations for gradient descent

    // Creating the GradientDescent object
    GradientDescent gd(initial_params, learning_rate, costFunction, {}, {});

    // Vectors to store the history of parameters and costs
    std::vector<double> param_history;
    std::vector<double> cost_history;

    // Perform gradient descent
    for (int i = 0; i < iterations; ++i)
    {
        gd.optimize(1); // Perform one iteration at a time to capture intermediate states

        // Fetching current parameters and cost
        double current_param = gd.getParameterAtIndex(0);
        double current_cost = costFunction({current_param});

        // Storing data for visualization
        param_history.push_back(current_param);
        cost_history.push_back(current_cost);

        std::cout << "Iteration: " << i << ", Parameter: " << current_param << ", Cost: " << current_cost << std::endl;
    }

    // Generate a range of parameters to plot the cost function curve
    std::vector<double> params_for_plot;
    std::vector<double> costs_for_plot;
    for (double p = 0; p <= 12; p += 0.1)
    {
        params_for_plot.push_back(p);
        costs_for_plot.push_back(costFunction({p}));
    }

    // Plotting the results
    plt::figure_size(1200, 900); // Set figure size
    plt::named_plot("Cost Function", params_for_plot, costs_for_plot, "r--");
    plt::named_plot("Gradient Descent Path", param_history, cost_history, "b-");
    plt::scatter(param_history, cost_history, 10.0, {{"color", "blue"}}); // Scatter points on path

    // Adding annotations at start and end
    plt::annotate("Start", param_history.front(), cost_history.front());
    plt::annotate("End", param_history.back(), cost_history.back());

    plt::xlabel("Parameter Value");
    plt::ylabel("Cost");
    plt::title("Gradient Descent Optimization Visualization");
    plt::legend();
    plt::show();

    return 0;
}
