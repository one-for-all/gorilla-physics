#include <gorilla/particle.h>

using namespace gorilla;

/////////////////////////////////////////////////////
void Particle::integrate(real duration)
{
  assert(duration >= 0.0);

  this->position += this->velocity*duration;

  Vector3 resultingAcc = this->acceleration;
  this->velocity += resultingAcc*duration;

  // Damp with duration seconds
  this->velocity *= pow(this->damping, duration);
}

/////////////////////////////////////////////////////
void Particle::setPosition(const Vector3 &position)
{
  this->position = position;
}

/////////////////////////////////////////////////////
void Particle::setVelocity(const Vector3 &velocity)
{
  this->velocity = velocity;
}

/////////////////////////////////////////////////////
void Particle::setAcceleration(const Vector3 &acceleration)
{
  this->acceleration = acceleration;
}

/////////////////////////////////////////////////////
Vector3 Particle::getPosition() const
{
  return this->position;
}

/////////////////////////////////////////////////////
Vector3 Particle::getVelocity() const
{
  return this->velocity;
}

/////////////////////////////////////////////////////
void Particle::setMass(const real mass)
{
  assert(mass > 0);
  this->inverseMass = 1.0/mass;
}

/////////////////////////////////////////////////////
void Particle::setDamping(const real damping)
{
  this->damping = damping;
}
