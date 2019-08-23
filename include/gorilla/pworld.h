#ifndef GORILLA_PWORLD_H
#define GORILLA_PWORLD_H

#include "pfgen.h"
#include "plinks.h"

namespace gorilla {

/// \brief An environment for simulating particles
class ParticleWorld
{
public:
  typedef std::vector<Particle*> Particles;
  typedef std::vector<ParticleContactGenerator*> ContactGenerators;

protected:
  /// \brief Particles in the world
  Particles particles;

  /// \brief Force registry for particles
  ParticleForceRegistry registry;

  /// \brief Contact resolver for particles
  ParticleContactResolver resolver;

  /// \brief Particle contact generators
  ContactGenerators contactGenerators;

  /// \brief List of contacts
  ParticleContact *contacts;

  /// \brief Max num of contacts
  std::size_t maxContacts;

public:
  /// \brief Constructor
  ParticleWorld(std::size_t maxContacts);

  /// \brief Destructor
  ~ParticleWorld();

  /// \brief Generate contacts in the world
  /// \return Number of contacts generated
  std::size_t generateContacts();

  /// \brief Integrate the dynamics of all the particles for the given duration
  void integrate(real duration);

  /// \brief Process all physics in the particle world
  void runPhysics(real duration);

  /// \brief Initialize the world for the next frame
  void startFrame();

  /// \brief Get all the particles
  Particles& getParticles();

  /// \brief Get all contact generators
  ContactGenerators& getContactGenerators();

  /// \brief Get force registry
  ParticleForceRegistry& getForceRegistry();

};
}

#endif // GORILLA_PWORLD_H