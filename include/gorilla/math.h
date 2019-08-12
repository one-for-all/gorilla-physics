#ifndef GORILLA_MATH_H
#define GORILLA_MATH_H

#include <Eigen/Dense>

namespace gorilla {

/// \brief precision definitions
#ifndef SINGLE_PRECISION
typedef double real;
#else
typedef float real;
#endif

/// \brief Math types
typedef Eigen::Matrix<real, 3, 1> Vector3;

/// \brief Physics constants
#define GRAVITY Vector3(0, -9.81, 0)

} // namespace gorilla

#endif // GORILLA_MATH_H
