#include <gorilla/pfgen.h>

using namespace gorilla;

///////////////////////////////////////////////////////////
void ParticleForceRegistry::updateForces(real duration)
{
  for (auto &registry : this->registrations)
  {
    registry.fg->updateForce(registry.particle, duration);
  }
}

///////////////////////////////////////////////////////////
void ParticleForceRegistry::add(Particle* particle, ParticleForceGenerator *fg)
{
  this->registrations.push_back({particle, fg});
}
