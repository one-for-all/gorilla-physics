#ifndef GORILLA_FGEN_H
#define GORILLA_FGEN_H

#include "body.h"

#include <vector>

namespace gorilla {

/// \brief Force generator to add force to bodies
class ForceGenerator
{
  /// \brief Calculate and update force applied to the rigid body
  public: virtual void updateForce(RigidBody *body, real duration) = 0;
};

/// \brief Registry for force generators and their applied bodies
class ForceRegistry
{
  protected: struct ForceRegistration
  {
    RigidBody *body;
    ForceGenerator *fg;
  };

  /// \brief List of force registrations
  protected: std::vector<ForceRegistration> registrations;

  /// \brief Register the force generator to the given body
  public: void add(RigidBody* body, ForceGenerator *fg);

  /// \brief Remove the registration of force generator to the body
  public: void remove(RigidBody *body, ForceGenerator *fg);

  /// \brief Clear all registrations
  public: void clear();

  /// \brief Update forces for all registrations
  public: void updateForces(real duration);
};
}

#endif // GORILLA_FGEN_H