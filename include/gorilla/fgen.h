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

/// \brief Aerodynamic force generator
class Aero : public ForceGenerator
{
  /// \brief Aerodynamic tensor for the surface in body frame
  protected: Matrix3 tensor;

  /// \brief Relative position of aerodynamic surface in body frame
  protected: Vector3 position;

  /// \brief Pointer to the wind speed of the environment
  protected: const Vector3* windSpeed;

  /// \brief Use explicit tensor matrix to update force
  protected: void updateForceFromTensor(RigidBody *body, const real duration,
                                        const Matrix3 &tensor);

  /// \brief Constructor
  public: Aero(const Matrix3 &tensor, const Vector3 &position,
               const Vector3 *windSpeed);

  // Documentation inherited
  public: virtual void updateForce(RigidBody *body, const real duration);
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