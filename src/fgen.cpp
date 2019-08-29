#include <gorilla/fgen.h>

using namespace gorilla;

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

