#include <gorilla/body.h>
#include <memory.h>
#include <assert.h>

using namespace gorilla;

/////////////////////////////////////////////////////////////////////
void RigidBody::setMass(const real mass)
{
  assert(mass > 0);
  this->inverseMass = 1.0/mass;
}

/////////////////////////////////////////////////////////////////////
void RigidBody::setInertiaTensor(const Matrix3 tensor)
{
  this->inverseInertiaTensor = tensor.inverse();
}

/////////////////////////////////////////////////////////////////////
void RigidBody::setDamping(const real linear, const real angular)
{
  this->linearDamping = linear;
  this->angularDamping = angular;
}

/////////////////////////////////////////////////////////////////////
void RigidBody::setAcceleration(const Vector3 acc)
{
  this->acceleration = acc;
}

/////////////////////////////////////////////////////////////////////
void RigidBody::calculateDerivedData()
{
  this->orientation.normalize();

  // Calculate the transform matrix from position and orientation
  this->transformMatrix.setIdentity();
  this->transformMatrix.block<3, 1>(0, 3) = this->position;
  this->transformMatrix.block<3, 3>(0, 0) =
    this->orientation.toRotationMatrix();

  // Calculate the inverse inertia tensor in world frame
  const auto &rotation = this->transformMatrix.block<3, 3>(0, 0);
  this->inverseInertiaTensorWorld = rotation *
                                    this->inverseInertiaTensor *
                                    rotation.transpose();
}

/////////////////////////////////////////////////////////////////////
void RigidBody::integrate(const real duration)
{
  this->lastFrameAcceleration = this->acceleration;
  this->lastFrameAcceleration += this->forceAccum * this->inverseMass;

  const Vector3 angularAcceleration = this->inverseInertiaTensorWorld *
                                      this->torqueAccum;

  // Update velocities
  this->velocity += this->lastFrameAcceleration * duration;
  this->angularVelocity += angularAcceleration * duration;

  // Apply damping
  this->velocity *= real_pow(this->linearDamping, duration);
  this->angularVelocity *= real_pow(this->angularDamping, duration);

  // Update pose
  this->position += this->velocity * duration;
  this->updateOrientation(angularVelocity, duration);

  this->calculateDerivedData();
  this->clearAccumulators();
}

/////////////////////////////////////////////////////////////////////
void RigidBody::setPosition(const Vector3 &pos)
{
  this->position = pos;
}

/////////////////////////////////////////////////////////////////////
void RigidBody::setPosition(const real x, const real y, const real z)
{
  this->position = Vector3(x, y, z);
}

/////////////////////////////////////////////////////////////////////
Vector3 RigidBody::getPosition() const
{
  return this->position;
}

/////////////////////////////////////////////////////////////////////
void RigidBody::setOrientation(const real w, const real x, const real y,
                               const real z)
{
  this->orientation = Quaternion(w, x, y, z);
  this->orientation.normalize();
}

/////////////////////////////////////////////////////////////////////
void RigidBody::updateOrientation(const Vector3 &angular, const real duration)
{
  auto rotation = Quaternion(0,
                             angular.x() * duration,
                             angular.y() * duration,
                             angular.z() * duration);
  rotation *= this->orientation;
  this->orientation.w() += rotation.w() * 0.5;
  this->orientation.x() += rotation.x() * 0.5;
  this->orientation.y() += rotation.y() * 0.5;
  this->orientation.z() += rotation.z() * 0.5;
}

/////////////////////////////////////////////////////////////////////
void RigidBody::setVelocity(const real x, const real y, const real z)
{
  this->velocity = Vector3(x, y, z);
}

/////////////////////////////////////////////////////////////////////
Vector3 RigidBody::getVelocity() const
{
  return this->velocity;
}

/////////////////////////////////////////////////////////////////////
void RigidBody::setAngularVelocity(const real x, const real y, const real z)
{
  this->angularVelocity = Vector3(x, y, z);
}

/////////////////////////////////////////////////////////////////////
Matrix4 RigidBody::getTransform() const
{
  return this->transformMatrix;
}

/////////////////////////////////////////////////////////////////////
void RigidBody::clearAccumulators()
{
  this->forceAccum.setZero();
  this->torqueAccum.setZero();
}

/////////////////////////////////////////////////////////////////////
void RigidBody::addForce(const Vector3 &force)
{
  this->forceAccum += force;
}

/////////////////////////////////////////////////////////////////////
void RigidBody::addForceAtPoint(const Vector3 &force,
                                    const Vector3 &point)
{
  const auto pt = point - this->position;

  this->forceAccum += force;
  this->torqueAccum += pt.cross(force);
}

/////////////////////////////////////////////////////////////////////
void RigidBody::addForceAtBodyPoint(const Vector3 &force,
                                    const Vector3 &point)
{
  const auto &pointWorld = this->getPointInWorldFrame(point);
  this->addForceAtPoint(force, pointWorld);
}

/////////////////////////////////////////////////////////////////////
Vector3 RigidBody::getPointInWorldFrame(const Vector3 &point) const
{
  Vector4 pointHomo;
  pointHomo << point, 1;
  return (this->transformMatrix * pointHomo).block<3, 1>(0, 0);
}