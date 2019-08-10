#ifndef GORILLA_PARTICLE_H
#define GORILLA_PARTICLE_H

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
      Vector3 forceAccum;

    public:
      void integrate(real duration);
      void setPosition(const Vector3 &position);
      void setVelocity(const Vector3 &velocity);
      void setAcceleration(const Vector3 &acceleration);
      Vector3 getPosition() const;
      Vector3 getVelocity() const;
      void setMass(const real mass);
      void setDamping(const real damping);
      void clearAccumulator();
  };
}

#endif // GORILLA_PARTICLE_H