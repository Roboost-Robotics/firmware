#include <cmath>
#include <iostream>
#include <matplotlibcpp.h>
#include <roboost/utils/filters.hpp>
#include <vector>

namespace plt = matplotlibcpp;

int main()
{
    // Sample rate and frequency settings
    double fs = 1000; // Sample rate in Hz
    double f = 5;     // Frequency of the sine wave
    int N = 1000;     // Number of samples

    // Time vector for one second of data
    std::vector<double> t(N);
    std::vector<double> signal(N);
    std::vector<double> noise(N);
    std::vector<double> noisy_signal(N);

    // Initialize filters
    roboost::filters::LowPassFilter lp(5.0, 1.0 / fs);
    roboost::filters::ExponentialMovingAverageFilter ema(0.1);
    roboost::filters::MovingAverageFilter ma(50);
    roboost::filters::IIRFilter iir(0.1, 0.1, 0, -0.3, 0.3);

    // Create a sine wave and add noise
    for (int i = 0; i < N; ++i)
    {
        t[i] = i / fs;
        signal[i] = sin(2 * M_PI * f * t[i]);
        noise[i] = 0.3 * ((double)rand() / RAND_MAX - 0.5); // Noise range -0.15 to 0.15
        noisy_signal[i] = signal[i] + noise[i];
    }

    // Filter the noisy signal
    std::vector<double> lp_output(N);
    std::vector<double> ema_output(N);
    std::vector<double> ma_output(N);
    std::vector<double> iir_output(N);

    for (int i = 0; i < N; ++i)
    {
        lp_output[i] = lp.update(noisy_signal[i]);
        ema_output[i] = ema.update(noisy_signal[i]);
        ma_output[i] = ma.update(noisy_signal[i]);
        iir_output[i] = iir.update(noisy_signal[i]);
    }

    // Plotting the results
    plt::figure_size(1200, 900); // Set figure size
    plt::subplot(3, 1, 1);
    plt::plot(t, noisy_signal, {{"label", "Noisy Signal"}});
    plt::title("Noisy Signal");
    plt::legend();

    plt::subplot(3, 1, 2);
    plt::plot(t, lp_output, {{"label", "Low Pass Filter"}});
    plt::plot(t, ema_output, {{"label", "Exponential Moving Average"}});
    plt::plot(t, ma_output, {{"label", "Moving Average Filter"}});
    plt::title("Filtered Signals");
    plt::legend();

    plt::subplot(3, 1, 3);
    plt::plot(t, iir_output, {{"label", "IIR Filter"}});
    plt::title("IIR Filter Output");
    plt::legend();

    // TODO: add more

    plt::show();

    return 0;
}
