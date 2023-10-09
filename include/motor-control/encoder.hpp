/**
 * @file encoder.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief This file contains the abstract base class for reading encoder values.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef ENCODER_H
#define ENCODER_H

#include "utils/constants.h"
#include <Arduino.h>
#include <ESP32Encoder.h>

/**
 * @brief Encoder base class.
 *
 */
class Encoder
{
public:
    /**
     * @brief Get the velocity of the encoder.
     *
     * @return float The velocity in rad/s.
     */
    virtual double get_velocity() = 0;

    /**
     * @brief Get the angle of the encoder.
     *
     * @return float The angle in rad.
     */
    virtual double get_angle() = 0;

    /**
     * @brief Update the encoder values.
     *
     * @note This function should be called regularly to update the encoder
     * values.
     */
    virtual void update() = 0;
};

/**
 * @brief Encoder class for quadrature encoders.
 *
 */
class HalfQuadEncoder : public Encoder
{
public:
    /**
     * @brief Construct a new HalfQuadEncoder object
     *
     * @param pin_A The pin for the A channel.
     * @param pin_B The pin for the B channel.
     * @param resolution The resolution of the encoder.
     * @param reverse Whether the encoder is reversed.
     */
    HalfQuadEncoder(const u_int8_t& pin_A, const u_int8_t& pin_B,
                    const u_int16_t& resolution, const bool reverse = false);

    /**
     * @brief Get the velocity of the encoder.
     *
     * @return double The velocity in rad/s.
     */
    double get_velocity() override;

    /**
     * @brief Get the position of the encoder.
     *
     * @return double The position in rad (0 to 2*PI).
     */
    double get_angle() override;

    /**
     * @brief Update the encoder values.
     *
     * @note This function should be called regularly to update the encoder
     * values.
     */
    void update() override;

private:
    ESP32Encoder encoder_;
    const u_int16_t resolution_;
    const bool reverse_;
    double position_ = 0;     // in radians
    double velocity_ = 0;     // in radians per second
    unsigned long last_time_; // in microseconds
};

#endif // ENCODER_H
