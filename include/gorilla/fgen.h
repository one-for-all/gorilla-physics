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

/// \brief Aerodynamic force generator with controllable surface
class AeroControl : public Aero
{
  /// \brief Aerodynamic tensor when control at minimum
  protected: Matrix3 minTensor;

  /// \brief Aerodynamic tensor when control at maximum
  protected: Matrix3 maxTensor;

  /// \brief Current control value, in range [-1, 1]
  protected: real controlSetting;

  /// \brief Compute tensor value with current control value
  private: Matrix3 getTensor();

  /// \brief Constructor
  public: AeroControl(const Matrix3 &base, const Matrix3 &min,
                      const Matrix3 &max, const Vector3 &position,
                      const Vector3 *windSpeed);

  /// \brief Set control value
  public: void setControl(const real value);

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