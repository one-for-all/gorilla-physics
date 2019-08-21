#ifndef GORILLA_PFGEN_H
#define GORILLA_PFGEN_H

#include "math.h"
#include "particle.h"
#include <vector>

namespace gorilla
{

/// \brief A force generator for adding force to particles
class ParticleForceGenerator
{
public:
  /// \brief Calculate and update force applied to the particle
  virtual void updateForce(Particle *particle, real duration) = 0;
};

/// \brief Hold particle force generators and corresponding particles
class ParticleForceRegistry
{
protected:
  /// \brief Hold a particle force generator and its particle
  struct ParticleForceRegistration
  {
    Particle *particle;
    ParticleForceGenerator *fg;
  };

  /// \brief A list of particle force registrations
  std::vector<ParticleForceRegistration> registrations;

public:
  /// \brief Add particle w/ fg to the registry
  void add(Particle *particle, ParticleForceGenerator *fg);

  void remove(Particle *particle, ParticleForceGenerator *fg);

  void clear();

  /// \brief Update force with each particle force generator
  void updateForces(real duration);
};
} // namespace gorilla

#endif // GORILLA_PFGEN_H