#include <gorilla/pworld.h>

using namespace gorilla;

///////////////////////////////////////////////////////////////////
ParticleWorld::ParticleWorld(std::size_t maxContacts)
: maxContacts(maxContacts), resolver(2*maxContacts)
{
  this->contacts = new ParticleContact[maxContacts];
}

///////////////////////////////////////////////////////////////////
ParticleWorld::~ParticleWorld()
{
  delete[] this->contacts;
}

///////////////////////////////////////////////////////////////////
void ParticleWorld::startFrame()
{
  for(auto &particle : this->particles)
  {
    particle->clearAccumulator();
  }
}

///////////////////////////////////////////////////////////////////
std::size_t ParticleWorld::generateContacts()
{
  std::size_t limit = this->maxContacts;
  ParticleContact *nextContact = this->contacts;

  for (auto &contactGenerator : this->contactGenerators)
  {
    auto used = contactGenerator->addContact(nextContact, limit);
    limit -= used;
    nextContact += used;

    if (limit <= 0)
      break;
  }

  return this->maxContacts - limit;
}

///////////////////////////////////////////////////////////////////
void ParticleWorld::integrate(real duration)
{
  for (auto &particle : this->particles)
  {
    particle->integrate(duration);
  }
}

///////////////////////////////////////////////////////////////////
void ParticleWorld::runPhysics(real duration)
{
  this->registry.updateForces(duration);

  this->integrate(duration);

  auto usedContacts = this->generateContacts();

  if (usedContacts > 0)
  {
    this->resolver.setIterations(usedContacts * 2);
    this->resolver.resolveContacts(this->contacts, usedContacts, duration);
  }
}

///////////////////////////////////////////////////////////////////
ParticleWorld::Particles& ParticleWorld::getParticles()
{
  return this->particles;
}

///////////////////////////////////////////////////////////////////
ParticleWorld::ContactGenerators& ParticleWorld::getContactGenerators()
{
  return this->contactGenerators;
}

///////////////////////////////////////////////////////////////////
ParticleForceRegistry& ParticleWorld::getForceRegistry()
{
  return this->registry;
}

///////////////////////////////////////////////////////////////////
void GroundContactsGenerator::init(ParticleWorld::Particles *particles)
{
  this->particles = particles;
}

///////////////////////////////////////////////////////////////////
std::size_t GroundContactsGenerator::addContact(ParticleContact *contact,
                                                std::size_t limit) const
{
  std::size_t count = 0;
  for (auto &particle : *(this->particles))
  {
    real y = particle->getPosition().y();
    if (y < 0)
    {
      contact->contactNormal = gorilla::UP;
      contact->particles[0] = particle;
      contact->particles[1] = nullptr;
      contact->penetration = -y;
      contact->restitution = 0.2;
      contact++;
      count++;
    }

    if (count >= limit)
      return count;
  }

  return count;
}