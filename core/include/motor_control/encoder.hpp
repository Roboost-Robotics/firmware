#ifndef ENCODER_H
#define ENCODER_H

class Encoder {
public:
  Encoder(int pinA, int pinB);

  float readVelocity();

  // Other encoder-related functions...

private:
  int pinA_, pinB_;
  // Other encoder-related variables...
};

#endif // ENCODER_H