#include <assert.h>
#include <gorilla/particle.h>

using namespace gorilla;

/////////////////////////////////////////////////////
void Particle::integrate(real duration)
{
  if (inverseMass <= 0)
    return;
  assert(duration > 0.0);

  position += velocity*duration;

  Vector3 resultingAcc = acceleration;
  velocity += resultingAcc*duration;

  velocity *= pow(damping, duration);

  clearAccumulator();
}

/////////////////////////////////////////////////////
void Particle::setPosition(const Vector3 &position)
{
  Particle::position = position;
}

/////////////////////////////////////////////////////
void Particle::setVelocity(const Vector3 &velocity)
{
  Particle::velocity = velocity;
}

/////////////////////////////////////////////////////
void Particle::setAcceleration(const Vector3 &acceleration)
{
  Particle::acceleration = acceleration;
}

/////////////////////////////////////////////////////
Vector3 Particle::getPosition() const
{
  return position;
}

/////////////////////////////////////////////////////
Vector3 Particle::getVelocity() const
{
  return velocity;
}

/////////////////////////////////////////////////////
void Particle::setMass(const real mass)
{
  assert(mass != 0);
  Particle::inverseMass = ((real)1.0)/mass;
}

/////////////////////////////////////////////////////
void Particle::setDamping(const real damping)
{
  Particle::damping = damping;
}

/////////////////////////////////////////////////////
void Particle::clearAccumulator()
{
  forceAccum.setZero();
}