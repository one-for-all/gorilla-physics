#include <gorilla/physics.h>

using namespace gorilla;

//////////////////////////////////////////////////////////////////////////
Matrix3 gorilla::CuboidInertiaTensor(const Vector3 sizes, const real mass)
{
  Vector3 sizesSquared = sizes.cwiseProduct(sizes);
  Matrix3 inertiaTensor;
  inertiaTensor(0, 0) = 1.0/12 * mass * (sizesSquared.y() + sizesSquared.z());
  inertiaTensor(1, 1) = 1.0/12 * mass * (sizesSquared.x() + sizesSquared.z());
  inertiaTensor(2, 2) = 1.0/12 * mass * (sizesSquared.x() + sizesSquared.y());

  return inertiaTensor;
}