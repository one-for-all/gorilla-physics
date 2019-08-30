#include <gorilla/fgen.h>
#include <gorilla/math.h>

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
void Aero::updateForceFromTensor(RigidBody *body, const real /* duration */,
                                 const Matrix3 &tensor)
{
  auto velocity = body->getVelocity();
  velocity += *(this->windSpeed);

  const auto &rotation = body->getTransform().block<3, 3>(0, 0);
  const auto &force = rotation * tensor * rotation.inverse() * velocity;

  body->addForceAtBodyPoint(force, this->position);
}

/////////////////////////////////////////////////////////////////////
AeroControl::AeroControl(const Matrix3 &base, const Matrix3 &min,
                         const Matrix3 &max, const Vector3 &position,
                         const Vector3 *windSpeed)
: Aero(base, position, windSpeed), minTensor(min), maxTensor(max),
  controlSetting(0)
{
}

/////////////////////////////////////////////////////////////////////
Matrix3 AeroControl::getTensor()
{
  if (this->controlSetting <= -1)
    return this->minTensor;

  if (this->controlSetting >= 1)
    return this->maxTensor;

  if (this->controlSetting < 0)
  {
    return matrixLinearInterpolate(this->minTensor, this->tensor,
                                   this->controlSetting+1);
  }
  else
  {
    return matrixLinearInterpolate(this->tensor, this->maxTensor,
                                   this->controlSetting);
  }
}

/////////////////////////////////////////////////////////////////////
void AeroControl::setControl(const real value)
{
  this->controlSetting = value;
}

/////////////////////////////////////////////////////////////////////
void AeroControl::updateForce(RigidBody *body, const real duration)
{
  const auto &tensor = this->getTensor();
  this->updateForceFromTensor(body, duration, tensor);
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

