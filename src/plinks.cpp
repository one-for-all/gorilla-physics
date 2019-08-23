#include <cassert>

#include <gorilla/plinks.h>

using namespace gorilla;

/////////////////////////////////////////////////////
real ParticleLink::currentLength() const
{
  const auto relativePos = this->particles[0]->getPosition() -
                           this->particles[1]->getPosition();
  return relativePos.norm();
}

/////////////////////////////////////////////////////
std::size_t ParticleRod::addContact(ParticleContact *contact,
                                    std::size_t limit) const
{
  const auto currentLen = this->currentLength();

  if (currentLen == this->length)
    return 0;

  assert(limit >= 1);

  // Approximate rod link with collision between the particles
  contact->particles[0] = this->particles[0];
  contact->particles[1] = this->particles[1];

  const auto normal = (this->particles[1]->getPosition() -
                      this->particles[0]->getPosition()).normalized();

  if (currentLen > this->length)
  {
    contact->contactNormal = normal;
    contact->penetration = currentLen - this->length;
  }
  else
  {
    contact->contactNormal = -normal;
    contact->penetration = this->length - currentLen;
  }

  contact->restitution = 0;

  return 1;
}