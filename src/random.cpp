#include <cstdlib>
#include <ctime>
#include <gorilla/random.h>

using namespace gorilla;

Random::Random()
{
  seed(0);
}

Random::Random(unsigned seed)
{
  Random::seed(seed);
}

void Random::seed(unsigned s)
{
  if (s == 0)
    s = (unsigned)clock();
  
  for (unsigned i = 0; i < 17; ++i)
  {
    s = s * 2891336453 + 1;
    buffer[i] = s;
  }

  p1 = 0;
  p2 = 0;
}

unsigned Random::rotl(unsigned n, unsigned r)
{
  return (n << r) | (n >> (32 -r));
}

unsigned Random::rotr(unsigned n, unsigned r)
{
  return (n >> r) | (n << (32 - r));
}

unsigned Random::randomBits()
{
  unsigned result;
  result = buffer[p1] = rotl(buffer[p2], 13) + rotl(buffer[p1], 9);

  if (--p1 < 0)
    p1 = 16;
  if (--p2 < 0)
    p2 = 16;

  return result;
}

real Random::randomReal()
{
  unsigned bits = randomBits();

  union {
    real value;
    unsigned word;
  } convert;

  convert.word = (bits >> 9) | 0x3f800000;

  return convert.value - 1.0f;
}

real Random::randomReal(real min, real max)
{
  return randomReal() * (max-min) + min;
}

unsigned Random::randomInt(unsigned max)
{
  return randomBits() % max;
}

Vector3 Random::randomVector(const Vector3 &min, const Vector3 &max)
{
  return Vector3(
    randomReal(min.x, max.x),
    randomReal(min.y, max.y),
    randomReal(min.z, max.z)
  );
}