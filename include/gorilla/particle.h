#include "core.h"

namespace gorilla {
  class Particle
  {
    protected:
      Vector3 position;
      Vector3 velocity;
      Vector3 acceleration;
      real damping;
      real inverseMass;

    public:
      void integrate(real duration);
  };
}
