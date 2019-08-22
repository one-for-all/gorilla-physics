#ifndef GORILLA_CONTACTS_H
#define GORILLA_CONTACTS_H

#include "particle.h"

namespace gorilla {

// Forward declaration, see full class below
class ParticleContactResolver;

class ParticleContact
{
friend class ParticleContactResolver;

public:
  /// \brief Particles involved in the contact.
  /// Second will be nullptr for contact with scene
  Particle* particles[2];

  /// \brief Restitution coefficient at the contact
  real restitution;

  /// \brief Direction of contact in world frame
  Vector3 contactNormal;

  /// \brief Depth of penetration at the contact
  real penetration;

  /// \brief Movements of particles during penetration resolution
  Vector3 particleMovements[2];

protected:
  /// \brief Resolve this contact, for both velocity and penetration
  void resolve(real duration);

  /// \brief Calculates the normal separating velocity at thie contact
  real calculateSeparatingVelocity() const;

private:
  /// \brief Resolve velocity update
  void resolveVelocity(real duration);

  /// \brief Resolve penetration
  void resolvePenetration(real duration);
};

/// \brief Resolver for particle contacts
class ParticleContactResolver
{
protected:
  /// \brief Max iterations allowed for resolving contacts
  std::size_t iterations;

  /// \brief Actual number of iterations used
  std::size_t iterationsUsed;

public:
  ParticleContactResolver(std::size_t iterations);

  /// \brief Set iterations allowed for resolving
  void setIterations(std::size_t iterations);

  /// \brief Resolve contacts for both velocity and penetration
  void resolveContacts(ParticleContact *contactArray, std::size_t numContacts, real duration);
};

/// \brief Interface for generating contacts
class ParticleContactGenerator
{
public:
  /// \brief Fills the contact array with generated contacts
  /// \param[in] limit Max number of contacts that can be added
  /// \return Number of contacts added
  virtual std::size_t addContact(ParticleContact *contact, std::size_t limit) const = 0;
};
}

#endif // GORILLA_CONTACTS_H