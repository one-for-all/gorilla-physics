#include <gorilla/gorilla.h>

class Firework: public gorilla::Particle
{
public:
  unsigned type;
  gorilla::real age;

  bool update(gorilla::real duration)
  {
    integrate(duration);
    age -= duration;
    return (age < 0) || (position.y < 0);
  }
};

struct FireworkRule
{
  unsigned type;
  gorilla::real minAge;
  gorilla::real maxAge;
  gorilla::Vector3 minVelocity;
  gorilla::Vector3 maxVelocity;
  gorilla::real damping;

  struct Payload
  {
    unsigned type;
    unsigned count;

    void set(unsigned type, unsigned count)
    {
      Payload::type = type;
      Payload::count = count;
    }
  }

  unsigned payloadCount;
  Payload *payload;

  void create(Firework *firework, const Firework *parent = nullptr) const
  {
    firework->type = type;
    firework->age = random.randomReal(minAge, maxAge);
    
  }
}