#include "precision.h"

namespace gorilla {
  class Random
  {
    unsigned randomBits();
    real randomReal();
    real randomReal(real min, real max);
  };
}