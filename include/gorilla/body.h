#ifndef GORILLA_BODY_H
#define GORILLA_BODY_H

#include "math.h"

namespace gorilla {

/// \brief Rigid body simulation object
class RigidBody
{
  /// \brief Inverse of mass
  protected: real inverseMass;

  /// \brief Inverse of inertia tensor in body frame
  protected: Matrix3 inverseInertiaTensor;

  /// \brief Inverse of inertia tensor in world frame
  protected: Matrix3 inverseInertiaTensorWorld;

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
  protected: Vector3 angularVelocity;

  /// \brief Transform matrix of the body
  protected: Matrix4 transformMatrix;

  /// \brief Force accumulator to be applied in the next frame
  protected: Vector3 forceAccum;

  /// \brief Torque accumulator to be applied in the next frame
  protected: Vector3 torqueAccum;

  /// \brief Linear acceleration of body when without forces
  /// Useful for setting gravity
  protected: Vector3 acceleration;

  /// \brief Computed linear acceleration in previous frame
  protected: Vector3 lastFrameAcceleration;

  /// \brief Set the mass of body
  public: void setMass(const real mass);

  /// \brief Set the inertia tensor of body
  public: void setInertiaTensor(const Matrix3 tensor);

  /// \brief Set linear and angular damping
  public: void setDamping(const real linear, const real angular);

  /// \brief Set linear acceleration
  public: void setAcceleration(const Vector3 acc);

  /// \brief Calculate internal data from state
  public: void calculateDerivedData();

  /// \brief Integrate rigid body dynamics forward in time
  public: void integrate(const real duration);

  /// \brief Set position by vector
  public: void setPosition(const Vector3 &pos);

  /// \brief Set position by component
  public: void setPosition(const real x, const real y, const real z);

  /// \brief Get position
  public: Vector3 getPosition() const;

  /// \brief Set orientation quaternion by component
  public: void setOrientation(const real w, const real x, const real y,
                              const real z);

  /// \brief Update orientation by angular velocity with duration
  public: void updateOrientation(const Vector3 &angular, const real duration);

  /// \brief Set linear velocity by component
  public: void setVelocity(const real x, const real y, const real z);

  /// \brief Get linear velocity
  public: Vector3 getVelocity() const;

  /// \brief Set angular velocity by component
  public: void setAngularVelocity(const real x, const real y, const real z);

  /// \brief Get transform matrix
  public: Matrix4 getTransform() const;

  /// \brief Clear force and torque accumulators
  public: void clearAccumulators();

  /// \brief Add force applied to the center of mass
  /// Force expressed in world frame
  public: void addForce(const Vector3 &force);

  /// \brief Add force applied at the position
  /// Both force and point given in world frame
  public: void addForceAtPoint(const Vector3 &force,
                               const Vector3 &point);

  /// \brief Add force applied at the position
  /// Force given in world frame, position in body frame
  public: void addForceAtBodyPoint(const Vector3 &force,
                                   const Vector3 &position);

  /// \brief Convert point from body frame to world frame
  public: Vector3 getPointInWorldFrame(const Vector3 &point) const;
};
}

#endif // GORILLA_BODY_H