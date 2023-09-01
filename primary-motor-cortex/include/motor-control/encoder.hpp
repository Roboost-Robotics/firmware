/**
 * @file encoder.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief //TODO
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <ESP32Encoder.h>

#define PI 3.1415926535897932384626433832795

/**
 * @brief Abstract base class for reading encoder values. //TODO
 *
 */
class Encoder
{
public:
    /**
     * @brief //TODO
     *
     * @return float
     */
    virtual float get_velocity() = 0;

    /**
     * @brief //TODO
     *
     * @return float
     */
    virtual float get_position() = 0;

    /**
     * @brief //TODO
     *
     */
    virtual void update() = 0;
};

/**
 * @brief //TODO
 *
 */
class HalfQuadEncoder : public Encoder
{
public:
    /**
     * @brief Construct a new Half Quad Encoder object //TODO
     *
     * @param pin_A
     * @param pin_B
     * @param resolution
     */
    HalfQuadEncoder(const int pin_A, const int pin_B, const int resolution);

    /**
     * @brief //TODO
     *
     * @return float
     */
    float get_velocity() override;

    /**
     * @brief //TODO
     *
     * @return float
     */
    float get_position() override;

    /**
     * @brief //TODO
     *
     */
    void update() override;

private:
    ESP32Encoder encoder_;
    int last_state_A_;
    int last_state_B_;
    const int resolution_;
    float position_ = 0;      // in radians
    float velocity_ = 0;      // in radians per second
    unsigned long last_time_; // in microseconds
};

#endif // ENCODER_H
