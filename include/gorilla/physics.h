#ifndef GORILLA_PHYSICS_H
#define GORILLA_PHYSICS_H

#include "math.h"

namespace gorilla {
/// \brief Gravity on Earth
#define GRAVITY Vector3(0, -9.81, 0)

Matrix3 CuboidInertiaTensor(const Vector3 sizes, const real mass);

} // namespace gorilla

#endif // GORILLA_PHYSICS_H
