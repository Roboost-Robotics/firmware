/**
 * @file encoder.hpp //todo
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef ENCODER_H
#define ENCODER_H

/**
 * @brief //todo
 *
 */
class Encoder
{
public:
    /**
     * @brief Construct a new Encoder object //todo
     *
     * @param pin_A
     * @param pin_B
     */
    Encoder(int pin_A, int pin_B);

    /**
     * @brief //todo
     *
     * @return float
     */
    float read_velocity();

private:
    int pin_A_, pin_B_;
};

#endif // ENCODER_H