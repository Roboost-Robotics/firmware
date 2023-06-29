#ifndef ENCODER_H
#define ENCODER_H

class Encoder {
public:
  Encoder(int pin_A, int pin_B);

  float read_velocity();

private:
  int pin_A_, pin_B_;
};

#endif // ENCODER_H