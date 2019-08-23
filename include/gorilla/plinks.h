#ifndef GORILLA_PLINKS_H
#define GORILLA_PLINKS_H

#include "pcontacts.h"

namespace gorilla {

class ParticleLink : public ParticleContactGenerator
{
protected:
  /// \brief Get current length of link
  real currentLength() const;

public:
  /// \brief Particles connected by the link
  Particle* particles[2];

  /// Documentation inherited
  virtual std::size_t addContact(ParticleContact *contact,
                                 std::size_t limit) const = 0;
};

class ParticleRod : public ParticleLink
{
public:
  /// \brief Length of rod
  real length;

  /// Documentation inherited
  virtual std::size_t addContact(ParticleContact *contact,
                                 std::size_t limit) const;
};
}

#endif // GORILLA_PLINKS_H