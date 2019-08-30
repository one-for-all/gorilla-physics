#include <gorilla/fgen.h>

using namespace gorilla;

/////////////////////////////////////////////////////////////////////
Aero::Aero(const Matrix3 &tensor, const Vector3 &position,
           const Vector3 *windSpeed)
: tensor(tensor), position(position), windSpeed(windSpeed)
{
}

/////////////////////////////////////////////////////////////////////
void Aero::updateForce(RigidBody *body, const real duration)
{
  this->updateForceFromTensor(body, duration, this->tensor);
}

/////////////////////////////////////////////////////////////////////
void Aero::updateForceFromTensor(RigidBody *body, const real duration,
                                 const Matrix3 &tensor)
{
  auto velocity = body->getVelocity();
  velocity += *(this->windSpeed);

  const auto &rotation = body->getTransform().block<3, 3>(0, 0);
  const auto &force = rotation * tensor * rotation.inverse() * velocity;

  body->addForceAtBodyPoint(force, this->position);
}

/////////////////////////////////////////////////////////////////////
void ForceRegistry::updateForces(real duration)
{
  for (auto &registration : this->registrations)
  {
    registration.fg->updateForce(registration.body, duration);
  }
}

/////////////////////////////////////////////////////////////////////
void ForceRegistry::add(RigidBody *body, ForceGenerator *fg)
{
  ForceRegistry::ForceRegistration registration;
  registration.body = body;
  registration.fg = fg;
  this->registrations.push_back(registration);
}

