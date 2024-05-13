#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

// Data structure to store the time and system output
struct DataPoint
{
    double time;   // Time in seconds
    double output; // Normalized measured output
};

// Function to read data from a CSV file
std::vector<DataPoint> readCSV(const std::string& filename)
{
    std::vector<DataPoint> data;
    std::ifstream file(filename);
    std::string line, cell;

    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return data;
    }

    std::getline(file, line);    // Skip the header line
    double finalSetPoint = 25.9; // Final setpoint or maximum expected output for normalization

    while (std::getline(file, line))
    {
        std::istringstream linestream(line);
        DataPoint point;

        std::getline(linestream, cell, ','); // Read timestamp
        if (!cell.empty() && cell.front() == '"')
            cell = cell.substr(1, cell.size() - 2); // Strip double quotes
        point.time = std::stod(cell) / 1000.0;      // Convert ms to seconds

        std::getline(linestream, cell, ','); // Read measured output
        if (!cell.empty() && cell.front() == '"')
            cell = cell.substr(1, cell.size() - 2); // Strip double quotes

        if (!cell.empty())
        {
            point.output = std::stod(cell) / finalSetPoint; // Normalize the output
            data.push_back(point);
        }
    }
    file.close();
    return data;
}

// Function to estimate parameters of a first-order system
std::pair<double, double> estimateFirstOrderParams(const std::vector<DataPoint>& data)
{
    double systemOutputFinal = data.back().output;
    double threshold = systemOutputFinal * (1 - exp(-1)); // 63.2% of final value
    double tau = 0;
    for (const auto& point : data)
    {
        if (point.output >= threshold)
        {
            tau = point.time;
            break;
        }
    }
    return {tau, systemOutputFinal}; // Return both time constant and gain
}

// Function to estimate parameters of a second-order system
std::tuple<double, double, double> estimateSecondOrderParams(const std::vector<DataPoint>& data)
{
    double maxOutput = std::max_element(data.begin(), data.end(), [](const DataPoint& a, const DataPoint& b) { return a.output < b.output; })->output;

    double systemOutputFinal = data.back().output;
    double overshoot = (maxOutput - systemOutputFinal) / systemOutputFinal;
    double dampingRatio = sqrt(log(overshoot) * log(overshoot) / (M_PI * M_PI + log(overshoot) * log(overshoot)));
    double peakTime = std::max_element(data.begin(), data.end(), [](const DataPoint& a, const DataPoint& b) { return a.output < b.output; })->time;
    double naturalFrequency = M_PI / (peakTime * sqrt(1 - dampingRatio * dampingRatio));

    return {systemOutputFinal, dampingRatio, naturalFrequency}; // Gain, damping ratio, natural frequency
}

int main()
{
    std::string filename = "res/teleplot_2024-5-8_22-20.csv";
    auto data = readCSV(filename);

    if (data.empty())
    {
        std::cerr << "No data read from file. Please check the file and try again." << std::endl;
        return 1;
    }

    auto [tau, gain] = estimateFirstOrderParams(data);
    auto [K, zeta, omega] = estimateSecondOrderParams(data);

    std::cout << "First-Order System Parameters:\n";
    std::cout << "Estimated Time Constant: " << tau << " seconds\n";
    std::cout << "Estimated Gain: " << gain << std::endl;

    std::cout << "Second-Order System Parameters:\n";
    std::cout << "Estimated Gain: " << K << std::endl;
    std::cout << "Estimated Damping Ratio: " << zeta << std::endl;
    std::cout << "Estimated Natural Frequency: " << omega << " rad/s\n";

    return 0;
}
