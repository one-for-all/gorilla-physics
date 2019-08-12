#include <gorilla/random.h>

using namespace gorilla;

Random::Random() : rd(), gen(rd()), real_dist(0, 1), int_dist(0)
{
}

real Random::randomReal()
{
  return real_dist(gen);
}

real Random::randomReal(real min, real max)
{
  return randomReal() * (max-min) + min;
}

unsigned Random::randomInt(unsigned max)
{
  return int_dist(gen) % max;
}

Vector3 Random::randomVector(const Vector3 &min, const Vector3 &max)
{
  return Vector3(
    randomReal(min.x(), max.x()),
    randomReal(min.y(), max.y()),
    randomReal(min.z(), max.z())
  );
}