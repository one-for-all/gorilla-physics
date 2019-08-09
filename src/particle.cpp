#include <assert.h>
#include <gorilla/particle.h>

using namespace gorilla;

void Particle::integrate(real duration)
{
  if (inverseMass <= 0)
    return;
  assert(duration > 0.0);

  position.addScaledVector(velocity, duration);

  Vector3 resultingAcc = acceleration;
  velocity.addScaledVector(resultingAcc, duration);

  velocity *= real_pow(damping, duration);

  // clearAccumulator();
}