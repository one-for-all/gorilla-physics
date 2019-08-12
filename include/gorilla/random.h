#ifndef GORILLA_RANDOM_H
#define GORILLA_RANDOM_H

#include <random>

#include "math.h"

namespace gorilla {
  class Random
  {
  public:
    Random();

    /// \brief Return random floating-point value in [0, 1)
    real randomReal();

    /// \brief Return random floating-point value in [min, max)
    real randomReal(real min, real max);

    /// \brief Return random integer in [0, max);
    unsigned randomInt(unsigned max);

    /// \brief Return random vector in [min, max)
    Vector3 randomVector(const Vector3 &min, const Vector3 &max);
  
  private:
    /// \brief Device for obtaining seed
    std::random_device rd;

    /// \brief Standard mersenne_twister_engine
    std::mt19937 gen;

    /// \brief Uniform floating-point distribution
    std::uniform_real_distribution<real> real_dist;

    /// \brief Uniform integer distribution
    std::uniform_int_distribution<int> int_dist;
  };
}

#endif // GORILLA_RANDOM_H
