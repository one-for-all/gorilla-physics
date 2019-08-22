#include <gorilla/pcontacts.h>

using namespace gorilla;

/////////////////////////////////////////////////////////
void ParticleContact::resolve(real duration)
{
  this->resolveVelocity(duration);
  this->resolvePenetration(duration);
}

/////////////////////////////////////////////////////////
real ParticleContact::calculateSeparatingVelocity() const
{
  auto relativeVelocity = this->particles[0]->getVelocity();
  if (this->particles[1] != nullptr)
    relativeVelocity -= this->particles[1]->getVelocity();
  return relativeVelocity.dot(this->contactNormal);
}

/////////////////////////////////////////////////////////
void ParticleContact::resolveVelocity(real duration)
{
  const auto separatingVelocity = this->calculateSeparatingVelocity();

  // If separating, no need to resolve velocity
  if (separatingVelocity >= 0)
    return;

  // Compute new separating velocity due to collision
  auto newSepVelocity = -separatingVelocity * this->restitution;

  // Compute velocity build-up due to acceleration
  auto relativeAcceleration = this->particles[0]->getAcceleration();
  if (this->particles[1] != nullptr)
    relativeAcceleration -= this->particles[1]->getAcceleration();
  real accCausedVelocity = relativeAcceleration.dot(this->contactNormal) *
                           duration;

  // If acceleration caused closing velocity, remove it from the new separating velocity.
  // Helps handling resting contact
  if (accCausedVelocity < 0)
  {
    newSepVelocity += restitution * accCausedVelocity;

    // Make sure we did not turn separating velocity to closing velocity
    if (newSepVelocity < 0)
      newSepVelocity = 0;
  }

  const auto deltaVelocity = newSepVelocity - separatingVelocity;

  // Apply velocity update in reverse proportion to masses
  auto totalInverseMass = this->particles[0]->getInverseMass();
  if (this->particles[1] != nullptr)
    totalInverseMass += this->particles[1]->getInverseMass();

  if (totalInverseMass <= 0)
    return;

  const auto totalImpulse = deltaVelocity / totalInverseMass;
  const auto totalImpulseVector = this->contactNormal * totalImpulse;

  this->particles[0]->setVelocity(this->particles[0]->getVelocity() +
                                  totalImpulseVector * this->particles[0]->getInverseMass());
  if (this->particles[1] != nullptr)
  {
    this->particles[1]->setVelocity(this->particles[1]->getVelocity() -
                                    totalImpulseVector * this->particles[1]->getInverseMass());
  }
}

/////////////////////////////////////////////////////////
void ParticleContact::resolvePenetration(real duration)
{
  if (this->penetration <= 0)
    return;

  // Move particle in reverse proportion to mass
  auto totalInverseMass = this->particles[0]->getInverseMass();
  if (this->particles[1] != nullptr)
    totalInverseMass += this->particles[1]->getInverseMass();

  if (totalInverseMass <= 0)
    return;

  const auto movementPerIMass = this->penetration / totalInverseMass *
                                this->contactNormal;
  this->particleMovements[0] = movementPerIMass * this->particles[0]
                                                      ->getInverseMass();
  if (this->particles[1] != nullptr)
  {
    this->particleMovements[1] = -movementPerIMass * this->particles[1]
                                                         ->getInverseMass();
  }
  else
  {
    this->particleMovements[1].setZero();
  }

  this->particles[0]->setPosition(
      this->particles[0]->getPosition() + this->particleMovements[0]);
  if (this->particles[1] != nullptr)
  {
    this->particles[1]->setPosition(
        this->particles[1]->getPosition() + this->particleMovements[1]);
  }
}

/////////////////////////////////////////////////////////
ParticleContactResolver::ParticleContactResolver(std::size_t iterations)
    : iterations(iterations)
{
}

/////////////////////////////////////////////////////////
void ParticleContactResolver::setIterations(std::size_t iterations)
{
  this->iterations = iterations;
}

/////////////////////////////////////////////////////////
void ParticleContactResolver::resolveContacts(
    ParticleContact *contactArray, std::size_t numContacts, real duration)
{
  std::size_t i;
  this->iterationsUsed = 0;
  while (this->iterationsUsed < this->iterations)
  {
    real maxVel = REAL_MAX;
    std::size_t maxIndex = numContacts;
    for (i = 0; i < numContacts; ++i)
    {
      real sepVel = contactArray[i].calculateSeparatingVelocity();
      if (sepVel < maxVel && (sepVel < 0 || contactArray[i].penetration > 0))
      {
        maxVel = sepVel;
        maxIndex = i;
      }
    }

    if (maxIndex == numContacts)
      break;

    // Resolve most severe collision
    contactArray[maxIndex].resolve(duration);

    // Update penetrations for particles involved
    Vector3 *move = contactArray[maxIndex].particleMovements;
    for (i = 0; i < numContacts; ++i)
    {
      if (contactArray[i].particles[0] == contactArray[maxIndex].particles[0])
      {
        contactArray[i].penetration -= move[0].dot(
          contactArray[i].contactNormal);
      }
      else if (
        contactArray[i].particles[0] == contactArray[maxIndex].particles[1])
      {
        contactArray[i].penetration -= move[1].dot(
          contactArray[i].contactNormal);
      }

      if (contactArray[i].particles[1] != nullptr)
      {
        if (contactArray[i].particles[1] == contactArray[maxIndex].particles[0])
        {
          contactArray[i].penetration += move[0].dot(
            contactArray[i].contactNormal);
        }
        else if (
          contactArray[i].particles[1] == contactArray[maxIndex].particles[1])
        {
          contactArray[i].penetration += move[1].dot(
            contactArray[i].contactNormal);
        }
      }
    }

    this->iterationsUsed++;
  }
}