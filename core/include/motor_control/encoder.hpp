#ifndef ENCODER_H
#define ENCODER_H

class Encoder {
public:
  Encoder(int pinA, int pinB);

  float readVelocity();

private:
  int pinA_, pinB_;
};

#endif // ENCODER_H