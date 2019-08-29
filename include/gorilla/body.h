#ifndef GORILLA_BODY_H
#define GORILLA_BODY_H

#include "math.h"

namespace gorilla {

/// \brief Rigid body simulation object
class RigidBody
{
  /// \brief Inverse of mass
  protected: real inverseMass;

  /// \brief Inverse of inertia tensor
  protected: Matrix3 inverseInertiaTensor;

  /// \brief Amount of linear velocity damping
  protected: real linearDamping;

  /// \brief Amount of angular velocity damping
  protected: real angularDamping;

  /// \brief Position of rigid body in world frame
  protected: Vector3 position;

  /// \brief Orientation of rigid body in world frame
  protected: Quaternion orientation;

  /// \brief Linear velocity in world frame
  protected: Vector3 velocity;

  /// \brief Angular velocity in world frame
  protected: Vector3 rotation;


};
}

#endif // GORILLA_BODY_H