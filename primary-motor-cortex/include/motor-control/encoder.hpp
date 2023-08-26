/**
 * @file encoder.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Defines the Encoder class, which provides rotational velocity reading
 * from an encoder.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

/**
 * @class Encoder
 * @brief Represents an encoder for rotational velocity measurement.
 *
 * This class provides functionality to read the rotational velocity from an
 * encoder with two input pins (A and B). It calculates the rotational velocity
 * based on changes in encoder state over time.
 */
class Encoder
{
public:
    /**
     * @brief Constructor for creating an Encoder object.
     *
     * @param pin_A The pin connected to the A channel of the encoder.
     * @param pin_B The pin connected to the B channel of the encoder.
     * @param resolution_ Resolution of the encoder in steops per revolution.
     */
    Encoder(const int pin_A, const int pin_B, const int resolution);

    /**
     * @brief Read and calculate the rotational velocity from the encoder.
     *
     * This method reads the state changes of the encoder and calculates the
     * rotational velocity based on the time between state changes and the
     * encoder's resolution.
     *
     * @return The calculated velocity in some appropriate units.
     */
    float read_velocity();

private:
    const int pin_A_, pin_B_;
    const int resolution_;
    int last_state_A_;
    int last_state_B_;
    unsigned long last_time_;
};

#endif // ENCODER_H