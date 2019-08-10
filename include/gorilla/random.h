#ifndef GORILLA_RANDOM_H
#define GORILLA_RANDOM_H

#include "core.h"

namespace gorilla {
  class Random
  {
  public:
    unsigned rotl(unsigned n, unsigned r);
    unsigned rotr(unsigned n, unsigned r);

    Random();
    Random(unsigned seed);

    void seed(unsigned seed);

    unsigned randomBits();
    real randomReal();
    real randomReal(real min, real max);
    unsigned randomInt(unsigned max);
    Vector3 randomVector(const Vector3 &min, const Vector3 &max);
  
  private:
    int p1, p2;
    unsigned buffer[17];
  };
}

#endif // GORILLA_RANDOM_H