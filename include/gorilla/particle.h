#ifndef GORILLA_PARTICLE_H
#define GORILLA_PARTICLE_H

#include "math.h"

namespace gorilla {

class Particle
{
protected:
  /// \brief Position of particle
  Vector3 position;

  /// \brief Velocity of particle
  Vector3 velocity;

  /// \brief Acceleration of particle
  Vector3 acceleration;

  /// \brief Percentage of velocity to retain in 1 second
  /// for numerical stability
  real damping;

  /// \brief Inverse of mass of the particle
  /// so as to be able to represent infinity and avoid zero
  real inverseMass;

public:
  /// \brief Integrate particle dynamics for duration time
  void integrate(real duration);

  /// \brief Set particle position
  void setPosition(const Vector3 &position);

  /// \brief Set particle velocity
  void setVelocity(const Vector3 &velocity);

  /// \brief Set particle acceleration
  void setAcceleration(const Vector3 &acceleration);

  /// \brief Get particle current position
  Vector3 getPosition() const;

  /// \brief Get particle current velocity
  Vector3 getVelocity() const;

  /// \brief Set particle mass value
  void setMass(const real mass);

  /// \brief Set particle damping value
  void setDamping(const real damping);
};

} // namespace gorilla

#endif // GORILLA_PARTICLE_H